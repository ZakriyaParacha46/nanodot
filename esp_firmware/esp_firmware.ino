/*
 * ==========================================================
 * NANODOT — Combined firmware (MQTT single-bus version)
 * Sensors + Stepper movement/odometry + WiFi + MQTT
 * ==========================================================
 * Libraries needed (Library Manager):
 *   - WiFiManager (tzapu)
 *   - PubSubClient (Nick O'Leary)
 *   - ArduinoJson (v6+)
 *   - VL53L0X (Pololu)
 *   - Ticker (bundled with ESP8266 core)
 *
 * MESSAGE DESIGN — single shared topic "nanodot/bus":
 *   { "sender": "<bot_id|server>", "receiver": "<bot_id|server|all>",
 *     "type": "handshake|ping|motion|cmd", "ts": <ms>, "data": {...} }
 *
 * handshake (bot->server, once at connect):
 *   data: { wheel_diameter_mm, wheel_base_mm, steps_per_rev, burst_steps }
 * ping (bot->server, every SEND_INTERVAL_MS while idle):
 *   data: { x, y, theta, f, r, l, bat }   // bat is null until battery circuit is finalized
 * motion (bot->server, after every BURST_STEPS during a move):
 *   data: { cmd, dir, x, y, theta, f, r, l, bat }
 * cmd (server->bot, drives the bot):
 *   data: { cmd: "F"|"B"|"L"|"R"|"Q"|"E"|"S", steps }   // steps omitted for Q/E/S (S = hard stop)
 * ==========================================================
 */

#include <ESP8266WiFi.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <Ticker.h>
#include <math.h>

#define DEBUG true

// ================= PIN DEFINITIONS =================
// Shift register (stepper coils only)
#define DATA_PIN      14   // 595 SER
#define CLOCK_PIN     13   // 595 SRCLK
#define LATCH_PIN     12   // 595 RCLK

// I2C bus
#define SDA_PIN       4
#define SCL_PIN       5

// VL53L0X XSHUT lines (native GPIO)
#define XSHUT_FORWARD 2    // ALSO the onboard LED — see note below
#define XSHUT_RIGHT   16
#define XSHUT_LEFT    15   // onboard pull-up removed on sensor board — no boot conflict now

// Status LEDs
#define LED_WIFI      2    // = XSHUT_FORWARD. Only blinks BEFORE sensor init (see connectWiFi()).
#define LED_ACK       0    // separate external LED — wire this one up on GPIO0

// ================= MQTT / WiFi CONFIG =================
const char* MQTT_BROKER = "test.mosquitto.org"; // Mosquitto's public test broker
const int   MQTT_PORT   = 1883;
const char* MQTT_TOPIC  = "nanodot/bus";

String BOT_ID = "nanodot-unset"; // real value computed in setup() from MAC address

WiFiClient espClient;
PubSubClient mqtt(espClient);
WiFiManager wifiManager;
Ticker wifiBlinker;
Ticker ackBlinker;

// ================= SENSOR CONFIG =================
#define ADDR_FORWARD 0x30
#define ADDR_RIGHT   0x32
#define ADDR_LEFT    0x31

VL53L0X sensorForward, sensorRight, sensorLeft;

enum SensorID { SENS_FORWARD, SENS_RIGHT, SENS_LEFT };

// ================= MOTOR / ODOMETRY CONFIG =================
const float WHEEL_DIAMETER_MM = 69.0;   // confirmed (measured)
const float WHEEL_BASE_MM     = 96.0;   // confirmed (measured)
const int   STEPS_PER_REV     = 4096;   // 28BYJ-48 half-step, output shaft
const float DIST_PER_STEP_MM  = (PI * WHEEL_DIAMETER_MM) / STEPS_PER_REV;

const byte halfStep[8] = {
  0b1000, 0b1100, 0b0100, 0b0110,
  0b0010, 0b0011, 0b0001, 0b1001
};
int motor1Position = 0; // motor1 = LEFT wheel  (bits 7-4)
int motor2Position = 0; // motor2 = RIGHT wheel (bits 3-0)
byte outputState = 0;
unsigned long stepDelayUs = 1800;
int BURST_STEPS = 200; // move N steps, then send a motion message, repeat

// Your two motors are mounted mirrored to each other (normal for
// differential drive). Confirmed working value: -1.
const int MOTOR2_MIRROR = -1;

enum Direction { MOVE_FORWARD, MOVE_BACKWARD, ROTATE_LEFT, ROTATE_RIGHT };

// ================= POSE STATE =================
float poseX = 0, poseY = 0, poseTheta = 0; // mm, mm, radians
char lastCommandChar = '-';

bool busyMoving = false;
bool pendingCmdWaiting = false;
char pendingCmdChar = '-';
int pendingCmdSteps = 0;
volatile bool stopRequested = false;

// ==========================================================
// SHIFT REGISTER — stepper coils only
// ==========================================================
void write595(byte value) {
  digitalWrite(LATCH_PIN, LOW);
  shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, value);
  digitalWrite(LATCH_PIN, HIGH);
  outputState = value;
}

// Immediately de-energizes motor coils and flags any in-progress
// moveSteps() to abort at its next checkpoint. Also cancels
// anything queued, since a hard stop should cancel everything.
void hardStop() {
  write595(0);
  stopRequested = true;
  pendingCmdWaiting = false;
  if (DEBUG) Serial.println("[DEBUG] HARD STOP executed.");
}

void doHalfStep(int dirLeft, int dirRight) {
  int actualDirRight = dirRight * MOTOR2_MIRROR; // compensate for mirrored mounting

  if (dirLeft != 0) {
    motor1Position += dirLeft;
    if (motor1Position >= 8) motor1Position = 0;
    if (motor1Position < 0)  motor1Position = 7;
  }
  if (actualDirRight != 0) {
    motor2Position += actualDirRight;
    if (motor2Position >= 8) motor2Position = 0;
    if (motor2Position < 0)  motor2Position = 7;
  }
  byte m1 = halfStep[motor1Position];
  byte m2 = halfStep[motor2Position] << 4;
  write595(m1 | m2);
  delayMicroseconds(stepDelayUs);
}

void updateOdometry(int leftStepsSigned, int rightStepsSigned) {
  float dLeft  = leftStepsSigned  * DIST_PER_STEP_MM;
  float dRight = rightStepsSigned * DIST_PER_STEP_MM;
  float dCenter = (dLeft + dRight) / 2.0;
  float dTheta  = (dRight - dLeft) / WHEEL_BASE_MM;

  poseX += dCenter * cos(poseTheta + dTheta / 2.0);
  poseY += dCenter * sin(poseTheta + dTheta / 2.0);
  poseTheta += dTheta;
  while (poseTheta > PI)  poseTheta -= 2 * PI;
  while (poseTheta < -PI) poseTheta += 2 * PI;
}

const char* directionLabel(Direction dir) {
  switch (dir) {
    case MOVE_FORWARD:  return "FORWARD";
    case MOVE_BACKWARD: return "BACKWARD";
    case ROTATE_LEFT:   return "ROTATE_LEFT";
    case ROTATE_RIGHT:  return "ROTATE_RIGHT";
  }
  return "?";
}

// ==========================================================
// SENSORS (forward declared here, defined below, needed by MQTT senders)
// ==========================================================
uint16_t readSensor(SensorID id);

// ==========================================================
// MQTT MESSAGE SENDERS
// ==========================================================
void publishAndMaybeLog(JsonDocument &doc, const char* tag, bool logIt = true) {
  char buf[400];
  size_t n = serializeJson(doc, buf);
  mqtt.publish(MQTT_TOPIC, buf, n);
  if (DEBUG && logIt) {
    Serial.print("[DEBUG] SENT "); Serial.print(tag); Serial.print(": ");
    Serial.println(buf);
  }
}

void sendHandshake() {
  StaticJsonDocument<300> doc;
  doc["sender"] = BOT_ID;
  doc["receiver"] = "server";
  doc["type"] = "handshake";
  doc["ts"] = millis();
  JsonObject data = doc.createNestedObject("data");
  data["wheel_diameter_mm"] = WHEEL_DIAMETER_MM;
  data["wheel_base_mm"] = WHEEL_BASE_MM;
  data["steps_per_rev"] = STEPS_PER_REV;
  data["burst_steps"] = BURST_STEPS;
  publishAndMaybeLog(doc, "handshake");
}

void sendPing() {
  StaticJsonDocument<300> doc;
  doc["sender"] = BOT_ID;
  doc["receiver"] = "server";
  doc["type"] = "ping";
  doc["ts"] = millis();
  JsonObject data = doc.createNestedObject("data");
  data["x"] = poseX;
  data["y"] = poseY;
  data["theta"] = poseTheta;
  data["f"] = readSensor(SENS_FORWARD);
  data["r"] = readSensor(SENS_RIGHT);
  data["l"] = readSensor(SENS_LEFT);
  data["bat"] = nullptr; // TODO: populate once battery divider is finalized
  publishAndMaybeLog(doc, "ping", false);
}

void sendMotion(Direction dir) {
  StaticJsonDocument<350> doc;
  doc["sender"] = BOT_ID;
  doc["receiver"] = "server";
  doc["type"] = "motion";
  doc["ts"] = millis();
  JsonObject data = doc.createNestedObject("data");
  data["cmd"] = String(lastCommandChar);
  data["dir"] = directionLabel(dir);
  data["x"] = poseX;
  data["y"] = poseY;
  data["theta"] = poseTheta;
  data["f"] = readSensor(SENS_FORWARD);
  data["r"] = readSensor(SENS_RIGHT);
  data["l"] = readSensor(SENS_LEFT);
  data["bat"] = nullptr; // TODO: populate once battery divider is finalized
  publishAndMaybeLog(doc, "motion");
}

// Called once after every completed burst of steps.
void afterBurst(Direction dir) {
  sendMotion(dir);
}

// ==========================================================
// MOVE — public function: moveSteps(direction, steps)
// Runs in bursts of BURST_STEPS, sending a motion message
// between bursts instead of after every single step.
// ==========================================================
void moveSteps(Direction dir, int totalSteps) {
  busyMoving = true;
  stopRequested = false; // fresh move — clear any stale stop flag from before
  int dirLeft = 0, dirRight = 0;
  switch (dir) {
    case MOVE_FORWARD:  dirLeft = 1;  dirRight = 1;  break;
    case MOVE_BACKWARD: dirLeft = -1; dirRight = -1; break;
    case ROTATE_LEFT:   dirLeft = -1; dirRight = 1;  break;
    case ROTATE_RIGHT:  dirLeft = 1;  dirRight = -1; break;
  }

  int stepsDone = 0;
  while (stepsDone < totalSteps) {
    int burst = min(BURST_STEPS, totalSteps - stepsDone);
    for (int i = 0; i < burst; i++) {
      doHalfStep(dirLeft, dirRight);
    }
    updateOdometry(dirLeft * burst, dirRight * burst);
    afterBurst(dir);
    mqtt.loop();            // may set stopRequested via a hard stop received mid-move
    handleSerialCommands(); // may also set stopRequested via a serial 'S'
    stepsDone += burst;

    if (stopRequested) {
      stopRequested = false;
      break; // abort the move early — coils already de-energized by hardStop()
    }
  }
  busyMoving = false;
}

// ==========================================================
// ROTATE90 — exact 90-degree turn, step count from wheel geometry
// ==========================================================
void rotate90(Direction dir) {
  float arcMm = (PI / 2.0) * (WHEEL_BASE_MM / 2.0);
  int steps90 = round(arcMm / DIST_PER_STEP_MM);

  moveSteps(dir, steps90);

  float snapped = round(poseTheta / (PI / 2.0)) * (PI / 2.0);
  poseTheta = snapped;
  while (poseTheta > PI)  poseTheta -= 2 * PI;
  while (poseTheta < -PI) poseTheta += 2 * PI;

  if (DEBUG) {
    Serial.print("[DEBUG] rotate90 complete, steps90="); Serial.print(steps90);
    Serial.print(" theta snapped to "); Serial.println(poseTheta, 3);
  }
}

// ==========================================================
// SENSORS
// ==========================================================
bool initOneSensor(VL53L0X &sensor, int xshutPin, uint8_t address, const char* label) {
  digitalWrite(xshutPin, HIGH);
  delay(50);
  if (!sensor.init()) {
    Serial.print(label); Serial.println(": INIT FAILED");
    return false;
  }
  sensor.setTimeout(200);
  sensor.setAddress(address);
  sensor.startContinuous();
  Serial.print(label); Serial.println(": OK");
  return true;
}

void initSensors() {
  pinMode(XSHUT_FORWARD, OUTPUT);
  pinMode(XSHUT_RIGHT, OUTPUT);
  pinMode(XSHUT_LEFT, OUTPUT);

  digitalWrite(XSHUT_FORWARD, LOW);
  digitalWrite(XSHUT_RIGHT, LOW);
  digitalWrite(XSHUT_LEFT, LOW);
  delay(50);

  Wire.begin(SDA_PIN, SCL_PIN);

  initOneSensor(sensorForward, XSHUT_FORWARD, ADDR_FORWARD, "Forward");
  initOneSensor(sensorRight,   XSHUT_RIGHT,   ADDR_RIGHT,   "Right");
  initOneSensor(sensorLeft,    XSHUT_LEFT,    ADDR_LEFT,    "Left");
  // NOTE: XSHUT_FORWARD (GPIO2) is now HIGH permanently — LED_WIFI
  // must not be touched again after this point.
}

uint16_t readSensor(SensorID id) {
  int xshutPin;
  uint8_t address;
  VL53L0X* sensor;
  const char* label;

  switch (id) {
    case SENS_FORWARD: xshutPin = XSHUT_FORWARD; address = ADDR_FORWARD; sensor = &sensorForward; label = "Forward"; break;
    case SENS_RIGHT:   xshutPin = XSHUT_RIGHT;   address = ADDR_RIGHT;   sensor = &sensorRight;   label = "Right";   break;
    case SENS_LEFT:    xshutPin = XSHUT_LEFT;    address = ADDR_LEFT;    sensor = &sensorLeft;    label = "Left";    break;
    default: return 65535;
  }

  uint16_t value = sensor->readRangeContinuousMillimeters();

  if (value == 65535 || sensor->timeoutOccurred()) {
    if (DEBUG) { Serial.print("[DEBUG] Sensor error ("); Serial.print(label); Serial.println("), attempting one reconnect..."); }
  }

  return value;
}

// ==========================================================
// LED SIGNALS
// ==========================================================
bool wifiLedState = false;
void toggleWifiLed() {
  wifiLedState = !wifiLedState;
  digitalWrite(LED_WIFI, wifiLedState ? LOW : HIGH); // active-low onboard LED
}

void ackLedOff() {
  digitalWrite(LED_ACK, LOW);
}

void flashAckLed() {
  digitalWrite(LED_ACK, HIGH);
  ackBlinker.once(0.5, ackLedOff);
}

// ==========================================================
// WIFI
// ==========================================================
void connectWiFi() {
  pinMode(LED_WIFI, OUTPUT);
  wifiBlinker.attach(1.0, toggleWifiLed);

  wifiManager.autoConnect("Nanodot-Setup");

  wifiBlinker.detach();
  digitalWrite(LED_WIFI, HIGH);
  Serial.println("WiFi connected.");
}

void computeBotId() {
  String mac = WiFi.macAddress(); // e.g. "5C:CF:7F:12:34:56"
  mac.replace(":", "");
  String last5 = mac.substring(mac.length() - 5);
  BOT_ID = "nanodot-" + last5;
  Serial.print("BOT_ID = "); Serial.println(BOT_ID);
}

// ==========================================================
// MQTT — incoming message handler
// ==========================================================
void executeCommand(char c, int steps) {
  lastCommandChar = c;
  switch (c) {
    case 'F': moveSteps(MOVE_FORWARD, steps); break;
    case 'B': moveSteps(MOVE_BACKWARD, steps); break;
    case 'L': moveSteps(ROTATE_LEFT, steps); break;
    case 'R': moveSteps(ROTATE_RIGHT, steps); break;
    case 'Q': rotate90(ROTATE_LEFT); break;
    case 'E': rotate90(ROTATE_RIGHT); break;
    default:
      if (DEBUG) Serial.println("[DEBUG] Unknown cmd letter received.");
  }
}

void onMqttMessage(char* topic, byte* payload, unsigned int length) {
  StaticJsonDocument<400> doc;
  DeserializationError err = deserializeJson(doc, payload, length);
  if (err) return;

  const char* sender = doc["sender"] | "";
  const char* receiver = doc["receiver"] | "";
  const char* type = doc["type"] | "";

  if (BOT_ID.equals(sender)) return; // ignore our own messages echoed back on the shared topic
  if (!BOT_ID.equals(receiver) && strcmp(receiver, "all") != 0) return; // not addressed to us

  if (DEBUG) {
    char buf[400];
    serializeJson(doc, buf);
    Serial.print("[DEBUG] RECV: ");
    Serial.println(buf);
  }

  if (strcmp(type, "cmd") == 0) {
    const char* cmdStr = doc["data"]["cmd"] | "";
    if (strlen(cmdStr) == 0) return;

    if (cmdStr[0] == 'S') { hardStop(); return; } // always immediate, never queued

    int steps = doc["data"]["steps"] | 0;

    if (busyMoving) {
      // Defer — executing now would nest inside the current moveSteps() call.
      pendingCmdChar = cmdStr[0];
      pendingCmdSteps = steps;
      pendingCmdWaiting = true;
      if (DEBUG) Serial.println("[DEBUG] cmd received while busy — queued.");
    } else {
      executeCommand(cmdStr[0], steps);
    }
  }
}

void connectMQTT() {
  mqtt.setServer(MQTT_BROKER, MQTT_PORT);
  mqtt.setCallback(onMqttMessage);

  while (!mqtt.connected()) {
    Serial.print("Connecting to MQTT...");
    if (mqtt.connect(BOT_ID.c_str())) {
      Serial.println("connected.");
      mqtt.subscribe(MQTT_TOPIC);
      sendHandshake();
    } else {
      Serial.print("failed, rc="); Serial.print(mqtt.state());
      delay(1000);
    }
  }
}

// ==========================================================
// Serial command parser — kept for local testing alongside MQTT control
// ==========================================================
void handleSerialCommands() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  if (cmd.length() < 1) return;

  char c = cmd.charAt(0);

  if (c == 'S') { hardStop(); return; } // always immediate, never queued

  int steps = (c == 'Q' || c == 'E') ? 0 : cmd.substring(1).toInt();
  if (c != 'Q' && c != 'E' && steps <= 0) return;

  if (busyMoving) {
    pendingCmdChar = c;
    pendingCmdSteps = steps;
    pendingCmdWaiting = true;
    if (DEBUG) Serial.println("[DEBUG] serial cmd received while busy — queued.");
  } else {
    executeCommand(c, steps);
  }
}

// ==========================================================
void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(DATA_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(LATCH_PIN, OUTPUT);
  pinMode(LED_ACK, OUTPUT);
  digitalWrite(LED_ACK, LOW);
  write595(0);

  connectWiFi();
  computeBotId();
  initSensors();
  connectMQTT();

  Serial.println("Nanodot ready. MQTT-controlled, serial F/B/L/R<steps> and Q/E also work.");
}

unsigned long lastSend = 0;
const unsigned long SEND_INTERVAL_MS = 250;

void loop() {
  if (!mqtt.connected()) connectMQTT();
  mqtt.loop();

  handleSerialCommands();

  if (pendingCmdWaiting && !busyMoving) {
    pendingCmdWaiting = false;
    executeCommand(pendingCmdChar, pendingCmdSteps);
  }

  if (millis() - lastSend > SEND_INTERVAL_MS) {
    sendPing();
    lastSend = millis();
  }
}
