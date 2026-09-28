"""
==========================================================
NANODOT — Live Pygame simulation + control
==========================================================
- Subscribes to nanodot/bus, tracks any number of bots by sender id
- Shows each bot on the mat in inches (490 steps = 1 in), starting at
  (4, 9) facing +X / right = 0 deg, over a 1-inch grid
- Draws the bot's 2.7 x 3.5 in footprint to scale; the 3 sensor rays start
  from the sensors on its front / left / right edges
- Sensor hits become dots -> visual map of the env. Each new reading
  wipes the old dots it sees through; a parked bot keeps 1 dot per line
- Calibration box (green) drawn around the start; each reading is scored against
  it per sensor, and the panel suggests offset fixes
- Keyboard controls send cmd messages back over MQTT
- AUTO EXPLORE: self-driving mode that covers as much floor as it can without
  touching walls. Covered floor is shaded teal.

Controls:
  W / Up     -> Forward
  S / Down   -> Backward
  A / Left   -> Rotate left
  D / Right  -> Rotate right
  Q          -> Turn left 90 deg = STEPS_PER_90_DEG steps  (firmware Q/E 90-deg cmd not accurate yet)
  E          -> Turn right 90 deg = STEPS_PER_90_DEG steps
  [ / ]      -> STEPS_PER_90_DEG -/+ 10, live (tune until 4 x Q is exactly 360 deg on the mat)
  SPACE      -> HARD STOP
  +/-        -> increase/decrease step size for F/B/L/R
  RESET btn  -> clear the map; bot's current spot becomes the start (4, 9) facing 0
  ROTATE 360 -> one full left turn (4 x STEPS_PER_90_DEG steps)
  ZOOM -/+   -> zoom out/in (or mouse wheel)
  FULLSCREEN -> fill the display (Esc or WINDOW button to go back)
  HIDE BOX   -> hide the green calibration box and its stats
  AUTO EXPLORE -> built-in autopilot on/off (any driving key or SPACE takes back control)

pip install paho-mqtt pygame
==========================================================
"""

import json
import math
import textwrap
import time
import threading
import pygame
import paho.mqtt.client as mqtt

# ---------------- Config ----------------
MQTT_BROKER = "test.mosquitto.org"
MQTT_PORT = 1883
TOPIC = "nanodot/bus"
CLIENT_SENDER_ID = "pysim"

MAX_RANGE_MM = 2000       # readings at/above this (8190/8191 out of range, 65535 error) are ignored

# Mat coordinates, in inches. The mat's inch marks are the ground truth.
STEPS_PER_INCH = 490
START_IN = (4.0, 9.0)     # where the drive-wheel axle midpoint starts, facing +X (0 deg)
MM_PER_INCH = 25.4        # sensor readings are real mm
# Added to each sensor's reading. Calibrate with the box: the panel suggests new values.
READING_OFFSET_IN = {"f": -3.24, "r": -3.68, "l": -2.46}   # from the box calibration
DEFAULT_STEPS = STEPS_PER_INCH
WHEEL_TRACK_IN = 3.92    # effective wheel spacing, tuned so 4 x Q = 360 deg on the mat (tape: 4 in)
# Turning in place, each wheel runs a quarter circle of radius track/2 per 90 deg.
# Q/E send this; fine-tune live with [ / ] (slip can make the bot need a bit more).
STEPS_PER_90_DEG = round(STEPS_PER_INCH * math.pi * WHEEL_TRACK_IN / 4)   # 3.895 in -> 1499

# Firmware odometry constants, used until the bot's handshake arrives
FW_WHEEL_DIAMETER_MM = 69.0
FW_WHEEL_BASE_MM = 85.0
FW_STEPS_PER_REV = 4096

LINE_TOL_IN = 0.15        # a beam wipes old dots this close sideways to its path...
HIT_TOL_IN = 0.5          # ...that are closer than (new hit - this): it saw through them
DOT_SPACING_IN = 0.1      # skip a new dot this close to an existing one (keeps the map light)

WINDOWED_SIZE = (1100, 750)   # FULLSCREEN button switches to the display's own size
PANEL_W = 300
PX_PER_IN = 20                # starting zoom; ZOOM buttons / mouse wheel change it live
ZOOM_RANGE = (4, 100)         # px per inch

# Bot footprint (inches). The tracked point (START_IN) is the drive-wheel axle midpoint,
# which is also the point the bot turns around.
BOT_WIDTH_IN = 2.7
BOT_LENGTH_IN = 3.5
AXLE_TO_FRONT_IN = BOT_LENGTH_IN   # bot pivots on its rear edge -> axle at the back (measure: wheel centres to front edge)

# Where each sensor sits relative to the axle midpoint, (forward, left) in inches,
# and which way it points (deg, left = +). Front sensor on the front edge,
# side sensors in the middle of the left/right edges.
SENSOR_MOUNTS = {
    "f": (AXLE_TO_FRONT_IN, 0.0, 0),
    "r": (AXLE_TO_FRONT_IN - BOT_LENGTH_IN / 2, -BOT_WIDTH_IN / 2, -90),
    "l": (AXLE_TO_FRONT_IN - BOT_LENGTH_IN / 2, BOT_WIDTH_IN / 2, 90),
}
# rays, dots and panel text per sensor (right is pink so it can't be confused with the green box)
SENSOR_COLORS = {"f": (80, 160, 255), "r": (255, 90, 200), "l": (255, 170, 60)}
SENSOR_NAMES = {"f": "front", "l": "left", "r": "right"}

# Calibration box: the bot sits inside it, turns in place, and every reading is scored
# against where the walls should be. Size of a MacBook Pro 16" (2021): 35.57 x 24.81 cm.
SHOW_BOX = True
BOX_SIZE_IN = (35.57 / 2.54, 24.81 / 2.54)   # (along the bot's start heading, across it)
BOX_OFFSET_IN = (0.0, 0.0)   # box centre minus the bot body's centre at start, (forward, left)

# Autopilot (AUTO EXPLORE button): covers as much floor as it can see without touching walls.
CELL_IN = 1.0             # coverage / planning grid = the mat's inch squares
FREE_MAX_IN = 30          # a reading only proves "nothing here" this far out
WHEEL_HALF_W_IN = 0.2     # half a tyre's width, for the wheels' footprint
BODY_MARGIN_IN = 0.5      # moves/turns must keep the whole footprint this far from wall dots
MAX_MOVE_IN = 3.0         # longest single forward move before re-checking
FRONT_MARGIN_IN = 1.0     # never drive closer than this to what the front sensor sees
STOP_IN = 0.8             # mid-move hard stop if the front sensor sees closer than this (twice)
TURN_COST_IN = 4.0        # choosing where to go next: a 180 deg turn costs like 4 in of driving
SWEEP_VALUE_IN = 0.3      # ...and each sq in it could sweep there is worth 0.3 in of driving
SCAN_SPACING_IN = 8.0     # look around (full turn) again only this far from earlier look-arounds


# ---------------- Layout (redone when switching full screen) ----------------
def set_layout(w, h):
    """Window size -> map-view centre (where START_IN is drawn) and panel buttons."""
    global WINDOW_W, WINDOW_H, VIEW_CX, VIEW_CY
    global ZOOM_OUT_BTN, ZOOM_IN_BTN, FULL_BTN, BOX_BTN, SPIN_BTN, RESET_BTN, AUTO_BTN
    WINDOW_W, WINDOW_H = w, h
    VIEW_CX = PANEL_W + (w - PANEL_W) // 2
    VIEW_CY = h // 2
    bw, gap = PANEL_W - 30, 6
    third, half = (bw - 2 * gap) // 3, (bw - gap) // 2
    AUTO_BTN = pygame.Rect(15, h - 362, bw, 34)
    ZOOM_OUT_BTN = pygame.Rect(15, h - 320, third, 34)
    ZOOM_IN_BTN = pygame.Rect(15 + third + gap, h - 320, third, 34)
    FULL_BTN = pygame.Rect(15 + 2 * (third + gap), h - 320, bw - 2 * (third + gap), 34)
    BOX_BTN = pygame.Rect(15, h - 278, half, 34)
    SPIN_BTN = pygame.Rect(15 + half + gap, h - 278, bw - half - gap, 34)
    RESET_BTN = pygame.Rect(15, h - 236, bw, 34)


set_layout(*WINDOWED_SIZE)

# ---------------- Shared state ----------------
bots_lock = threading.Lock()
bots = {}  # id -> {x,y (odometry mm),theta,pos_in,f,r,l,bat,dims,last_seen,points:[(x_in,y_in,sensor),...]}

step_size = DEFAULT_STEPS
box_visible = SHOW_BOX   # HIDE BOX button: hides the green box + its stats; scoring carries on
px_per_in = PX_PER_IN    # current zoom
fullscreen = False

# ---------------- MQTT ----------------
client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)

def send_cmd(cmd, steps=None, receiver="all"):
    payload = {
        "sender": CLIENT_SENDER_ID,
        "receiver": receiver,
        "type": "cmd",
        "ts": int(time.time() * 1000),
        "data": {"cmd": cmd}
    }
    if steps is not None:
        payload["data"]["steps"] = steps
    client.publish(TOPIC, json.dumps(payload))


def integrate_odometry(bot, x, y, theta):
    """Move the sim's own pose (inches, rad) by how much the firmware pose changed
    since the last message. Distances go back through steps (490 steps/in); turns
    are rescaled to the measured STEPS_PER_90_DEG, since the firmware's wheel-base
    maths says 90 deg is ~1261 steps."""
    prev, bot["fw_prev"] = bot["fw_prev"], (x, y, theta)
    if prev is None:
        return  # first message, or the firmware just (re)connected: new reference only
    px, py, pth = prev
    dims = bot["dims"]
    mm_per_step = (math.pi * dims.get("wheel_diameter_mm", FW_WHEEL_DIAMETER_MM)
                   / dims.get("steps_per_rev", FW_STEPS_PER_REV))
    fw_rad_per_step = 2 * mm_per_step / dims.get("wheel_base_mm", FW_WHEEL_BASE_MM)

    d_fw = (theta - pth + math.pi) % (2 * math.pi) - math.pi   # firmware wraps at +-180
    mid = pth + d_fw / 2
    ds_in = ((x - px) * math.cos(mid) + (y - py) * math.sin(mid)) / mm_per_step / STEPS_PER_INCH
    if abs(ds_in) > 3 or abs(d_fw) > 1.0:
        return  # a jump no burst can make: firmware rebooted, so its pose restarted at 0
    d_heading = d_fw * (math.pi / 2 / STEPS_PER_90_DEG) / fw_rad_per_step

    h = bot["heading"] + d_heading / 2
    bot["pos_in"] = (bot["pos_in"][0] + ds_in * math.cos(h), bot["pos_in"][1] + ds_in * math.sin(h))
    bot["heading"] += d_heading


def reset_all():
    """Clear the map and box stats, and take every bot's current spot as START_IN
    facing 0 deg (put the bot back on its start mark first)."""
    with bots_lock:
        for bot in bots.values():
            bot["pos_in"], bot["heading"] = START_IN, 0.0
            bot["points"].clear()
            bot["dot_cells"].clear()
            bot["last_dot"].clear()
            bot["box_err"] = {k: [0, 0.0, 0.0] for k in SENSOR_MOUNTS}
            bot["covered"].clear()
            bot["free"].clear()
            bot["scans"].clear()


def heading_deg(theta):
    """Firmware theta (radians, 0 = facing +X/right, left turns positive) -> 0..360 deg."""
    return math.degrees(theta) % 360


def reading_in(dist_mm, key):
    """Raw reading (mm) from sensor `key` -> corrected distance from the sensor, in inches."""
    return max(min(dist_mm, MAX_RANGE_MM) / MM_PER_INCH + READING_OFFSET_IN[key], 0.0)


def box_rect():
    """Calibration box (x0, y0, x1, y1) in mat inches, square to the start heading and
    centred on the bot body's centre at start (plus BOX_OFFSET_IN)."""
    cx = START_IN[0] + AXLE_TO_FRONT_IN - BOT_LENGTH_IN / 2 + BOX_OFFSET_IN[0]
    cy = START_IN[1] + BOX_OFFSET_IN[1]
    w, h = BOX_SIZE_IN
    return cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2


def box_distance(x, y, beam_theta):
    """How far a beam from (x, y) should travel before hitting the box walls from
    inside, or None if (x, y) isn't inside the box."""
    x0, y0, x1, y1 = box_rect()
    if not (x0 < x < x1 and y0 < y < y1):
        return None
    ux, uy = math.cos(beam_theta), math.sin(beam_theta)
    ts = []
    if abs(ux) > 1e-9:
        ts.append(((x1 if ux > 0 else x0) - x) / ux)
    if abs(uy) > 1e-9:
        ts.append(((y1 if uy > 0 else y0) - y) / uy)
    return min(ts)


def score_against_box(bot, key, x, y, beam_theta, dist_mm):
    """Accumulate (reading - expected) for sensor `key` into bot["box_err"]."""
    if dist_mm is None or dist_mm >= MAX_RANGE_MM:
        return
    expected = box_distance(x, y, beam_theta)
    if expected is None:
        return
    err = reading_in(dist_mm, key) - expected
    stats = bot["box_err"][key]
    stats[0] += 1
    stats[1] += err
    stats[2] += err * err


def body_to_mat(x, y, theta, fwd, left):
    """A point fixed on the bot (inches forward/left of the axle midpoint) -> mat inches."""
    c, s = math.cos(theta), math.sin(theta)
    return (x + fwd * c - left * s, y + fwd * s + left * c)


def sensor_ray(bot, key):
    """Where sensor `key` sits on the mat and the direction it points (inches, radians)."""
    fwd, left, offset_deg = SENSOR_MOUNTS[key]
    sx, sy = body_to_mat(*bot["pos_in"], bot["heading"], fwd, left)
    return sx, sy, bot["heading"] + math.radians(offset_deg)


def update_ray(bot, key, x, y, beam_theta, dist_mm):
    """One reading from sensor `key`, sitting at (x, y) and pointing along beam_theta.
    - Out-of-range readings (8190/8191) are ignored: while moving, the sensors
      drop out like this every few readings even with a wall in view.
    - Old dots the beam passed through (closer than the new hit) are wiped.
      Dots behind the hit are kept, since the sensor can't see past it.
    - If the sensor hasn't moved since its last dot, that dot is replaced,
      so a parked bot keeps 1 dot per line.
    - The new hit is added unless a dot already sits within DOT_SPACING_IN."""
    if dist_mm is None or dist_mm >= MAX_RANGE_MM:
        return
    dist_in = reading_in(dist_mm, key)
    ux, uy = math.cos(beam_theta), math.sin(beam_theta)
    hit = (x + dist_in * ux, y + dist_in * uy, key)
    pose = (round(x, 4), round(y, 4), round(beam_theta, 4))
    prev_pose, prev_dot = bot["last_dot"].get(key, (None, None))
    stale = prev_dot if prev_pose == pose else None

    def seen_through(p):
        dx, dy = p[0] - x, p[1] - y
        along = dx * ux + dy * uy
        return 0 <= along < dist_in - HIT_TOL_IN and abs(dx * uy - dy * ux) <= LINE_TOL_IN

    # only dots in the cells along the beam can be seen through
    beam_cells = set()
    for i in range(int(max(dist_in, 0.0) / 0.5) + 1):
        cx, cy = cell_of(x + i * 0.5 * ux, y + i * 0.5 * uy)
        beam_cells.update((cx + dx, cy + dy) for dx in (-1, 0, 1) for dy in (-1, 0, 1))
    doomed = [p for p in dots_in_cells(bot, beam_cells) if seen_through(p)]
    if stale is not None:
        doomed.append(stale)
    remove_dots(bot, doomed)

    hc = cell_of(hit[0], hit[1])
    around_hit = [(hc[0] + dx, hc[1] + dy) for dx in (-1, 0, 1) for dy in (-1, 0, 1)]
    if any(math.hypot(p[0] - hit[0], p[1] - hit[1]) < DOT_SPACING_IN for p in dots_in_cells(bot, around_hit)):
        bot["last_dot"][key] = (pose, None)
    else:
        add_dot(bot, hit)
        bot["last_dot"][key] = (pose, hit)


def cell_of(x, y):
    return (math.floor(x / CELL_IN), math.floor(y / CELL_IN))


# Wall dots live in bot["points"] (a dict used as an ordered set) and, for fast lookups
# near a spot, in bot["dot_cells"] (grid cell -> set of dots). Change them only via these.
def add_dot(bot, p):
    bot["points"][p] = None
    bot["dot_cells"].setdefault(cell_of(p[0], p[1]), set()).add(p)


def remove_dots(bot, dots):
    for p in dots:
        if bot["points"].pop(p, 0) is None:          # it was there
            c = cell_of(p[0], p[1])
            cell = bot["dot_cells"].get(c)
            if cell is not None:
                cell.discard(p)
                if not cell:
                    del bot["dot_cells"][c]


def dots_in_cells(bot, cells):
    grid = bot["dot_cells"]
    return [p for c in cells for p in grid.get(c, ())]


def cell_center(cell):
    return ((cell[0] + 0.5) * CELL_IN, (cell[1] + 0.5) * CELL_IN)


def mark_body(bot):
    """Cells under the body: ground covered, and known to be free floor."""
    x, y = bot["pos_in"]
    rear = AXLE_TO_FRONT_IN - BOT_LENGTH_IN
    for i in range(int(BOT_LENGTH_IN / 0.5) + 1):
        for j in range(int(BOT_WIDTH_IN / 0.5) + 1):
            fwd = min(rear + i * 0.5, AXLE_TO_FRONT_IN)
            left = min(-BOT_WIDTH_IN / 2 + j * 0.5, BOT_WIDTH_IN / 2)
            cell = cell_of(*body_to_mat(x, y, bot["heading"], fwd, left))
            bot["covered"].add(cell)
            bot["free"].add(cell)


def mark_seen(bot, key, x, y, beam_theta, dist_mm, moving):
    """Cells a valid reading passed through are free floor. Stops well short of the hit
    (further when moving, when readings are noisier) so a long reading can't mark the
    inside of a wall's corner as free."""
    if dist_mm is None or dist_mm >= MAX_RANGE_MM:
        return
    reach = min(reading_in(dist_mm, key) - (1.5 if moving else 1.0), FREE_MAX_IN)
    ux, uy = math.cos(beam_theta), math.sin(beam_theta)
    for i in range(int(reach / 0.5) + 1):
        bot["free"].add(cell_of(x + i * 0.5 * ux, y + i * 0.5 * uy))


def on_message(c, userdata, msg):
    try:
        envelope = json.loads(msg.payload.decode())
    except Exception:
        return

    sender = envelope.get("sender", "")
    mtype = envelope.get("type", "")
    data = envelope.get("data", {})

    if sender == CLIENT_SENDER_ID:
        return  # ignore our own published commands echoed back

    with bots_lock:
        if sender not in bots:
            bots[sender] = {
                "x": 0, "y": 0, "theta": 0, "fw_prev": None,
                "pos_in": START_IN, "heading": 0.0,
                "f": MAX_RANGE_MM, "r": MAX_RANGE_MM, "l": MAX_RANGE_MM,
                "bat": None, "dims": {}, "last_seen": time.time(),
                "points": {}, "dot_cells": {}, "last_dot": {},
                "box_err": {k: [0, 0.0, 0.0] for k in SENSOR_MOUNTS},  # n, sum, sum of squares
                "covered": set(),   # grid cells the body has been over
                "free": set(),      # grid cells a sensor has seen through, or the body was on
                "scans": [],        # where the autopilot has looked around
                "motion_count": 0, "pings_since_motion": 0,   # autopilot: is a move running/done?
            }
        bot = bots[sender]
        bot["last_seen"] = time.time()

        if mtype == "handshake":
            bot["dims"] = data
            bot["fw_prev"] = None  # (re)connected, maybe rebooted: don't diff across it

        elif mtype in ("ping", "motion"):
            bot["x"] = data.get("x", bot["x"])
            bot["y"] = data.get("y", bot["y"])
            bot["theta"] = data.get("theta", bot["theta"])
            bot["f"] = data.get("f", bot["f"])
            bot["r"] = data.get("r", bot["r"])
            bot["l"] = data.get("l", bot["l"])
            bot["bat"] = data.get("bat", bot["bat"])
            if mtype == "motion":
                bot["last_cmd"] = data.get("cmd", "?")
                bot["last_dir"] = data.get("dir", "?")
                bot["motion_count"] += 1
                bot["pings_since_motion"] = 0
            else:
                bot["pings_since_motion"] += 1   # pings only come while the bot is idle

            integrate_odometry(bot, bot["x"], bot["y"], bot["theta"])
            mark_body(bot)
            # nothing can be where the bot is standing: dots inside its footprint are sensor noise
            ghosts = [p for p in dots_near(bot, *bot["pos_in"], REACH_IN)
                      if footprint_hits(*bot["pos_in"], bot["heading"], [p], 0.0)]
            remove_dots(bot, ghosts)

            for key in SENSOR_MOUNTS:
                ray = sensor_ray(bot, key)
                update_ray(bot, key, *ray, bot[key])
                mark_seen(bot, key, *ray, bot[key], moving=mtype == "motion")
                if SHOW_BOX:
                    score_against_box(bot, key, *ray, bot[key])


def on_connect(c, userdata, flags, reason_code, properties):
    print(f"MQTT connected to {MQTT_BROKER}: {reason_code}")
    c.subscribe(TOPIC)


def mqtt_thread():
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(MQTT_BROKER, MQTT_PORT)
    client.loop_forever()


# ---------------- Autopilot ----------------
def footprint_hits(x, y, heading, dots, margin):
    """Would the body or wheels, with the axle at (x, y) facing `heading`, touch any dot?"""
    c, s = math.cos(heading), math.sin(heading)
    wheel_r = FW_WHEEL_DIAMETER_MM / MM_PER_INCH / 2
    for (px, py, _) in dots:
        dx, dy = px - x, py - y
        fwd, left = dx * c + dy * s, -dx * s + dy * c
        if (AXLE_TO_FRONT_IN - BOT_LENGTH_IN - margin <= fwd <= AXLE_TO_FRONT_IN + margin
                and abs(left) <= BOT_WIDTH_IN / 2 + margin):
            return True
        if abs(fwd) <= wheel_r + margin and abs(abs(left) - WHEEL_TRACK_IN / 2) <= WHEEL_HALF_W_IN + margin:
            return True
    return False


NEAR_WALL_CELLS = 2       # unseen floor within this many cells of a wall dot might hide its corner


def near_wall_cells(bot):
    """Cells within NEAR_WALL_CELLS of any wall dot (cached until the map changes)."""
    key = (len(bot["points"]), bot["motion_count"], bot["pings_since_motion"])
    if bot.get("near_wall_key") != key:
        cells = set()
        for (cx, cy) in bot["dot_cells"]:
            for dx in range(-NEAR_WALL_CELLS, NEAR_WALL_CELLS + 1):
                for dy in range(-NEAR_WALL_CELLS, NEAR_WALL_CELLS + 1):
                    cells.add((cx + dx, cy + dy))
        bot["near_wall"], bot["near_wall_key"] = cells, key
    return bot["near_wall"]


def footprint_clear_of_unseen_walls(bot, x, y, heading):
    """Does every part of the footprint stay over floor that's either been seen clear, or is
    unseen but away from any known wall? Unseen floor next to a wall can hide a corner or the
    end of a wall that no beam has hit yet (a turn once clipped exactly such a corner)."""
    known, risky = bot["free"], near_wall_cells(bot)
    rear = AXLE_TO_FRONT_IN - BOT_LENGTH_IN
    wheel_r = FW_WHEEL_DIAMETER_MM / MM_PER_INCH / 2
    pts = [(min(rear + i * 0.5, AXLE_TO_FRONT_IN), min(-BOT_WIDTH_IN / 2 + j * 0.5, BOT_WIDTH_IN / 2))
           for i in range(int(BOT_LENGTH_IN / 0.5) + 1) for j in range(int(BOT_WIDTH_IN / 0.5) + 1)]
    pts += [(f, side * (WHEEL_TRACK_IN / 2 + WHEEL_HALF_W_IN)) for f in (-wheel_r, 0.0, wheel_r) for side in (-1, 1)]
    for f, l in pts:
        c = cell_of(*body_to_mat(x, y, heading, f, l))
        if c not in known and c in risky:
            return False
    return True


_offset_cache = {}


def clearance_offsets(clearance):
    """Grid offsets from a wall-dot cell: `sure` = every point of the dot cell is within
    `clearance` of that cell's centre, `maybe` = only some are (check the actual dots)."""
    if clearance not in _offset_cache:
        r = math.ceil(clearance / CELL_IN) + 1
        sure, maybe = [], []
        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                cx, cy = (dx + 0.5) * CELL_IN, (dy + 0.5) * CELL_IN     # neighbour centre, dot cell at [0, CELL_IN)^2
                nx, ny = min(max(cx, 0.0), CELL_IN), min(max(cy, 0.0), CELL_IN)
                near = math.hypot(cx - nx, cy - ny)
                far = max(math.hypot(cx - px, cy - py) for px in (0.0, CELL_IN) for py in (0.0, CELL_IN))
                if far <= clearance:
                    sure.append((dx, dy))
                elif near <= clearance:
                    maybe.append((dx, dy))
        _offset_cache[clearance] = (sure, maybe)
    return _offset_cache[clearance]


def dots_near(bot, x, y, radius):
    cx, cy = cell_of(x, y)
    r = math.ceil(radius / CELL_IN) + 1
    cells = [(cx + dx, cy + dy) for dx in range(-r, r + 1) for dy in range(-r, r + 1)]
    return [p for p in dots_in_cells(bot, cells) if math.hypot(p[0] - x, p[1] - y) <= radius]


# furthest point of the footprint from the pivot: a front/rear body corner or a wheel's outer edge
REACH_IN = max(math.hypot(AXLE_TO_FRONT_IN, BOT_WIDTH_IN / 2),
               math.hypot(BOT_LENGTH_IN - AXLE_TO_FRONT_IN, BOT_WIDTH_IN / 2),
               math.hypot(FW_WHEEL_DIAMETER_MM / MM_PER_INCH / 2, WHEEL_TRACK_IN / 2 + WHEEL_HALF_W_IN))
# the planner only routes the axle through cells this far from every wall dot, so the bot can
# always turn in place wherever it stops (narrow gaps it could enter but not turn in are avoided)
PASS_CLEAR_IN = REACH_IN + BODY_MARGIN_IN


def turn_is_clear(bot, delta, from_heading=None):
    """Turning in place by `delta` rad (from the current heading, or from_heading):
    does the swept body stay clear of every wall dot?"""
    x, y = bot["pos_in"]
    h0 = bot["heading"] if from_heading is None else from_heading
    near, tight = split_tight(bot, x, y, h0, REACH_IN + BODY_MARGIN_IN)
    n = max(1, math.ceil(abs(delta) / math.radians(4)))
    return not any(footprint_hits(x, y, h0 + delta * i / n, near, BODY_MARGIN_IN)
                   or footprint_hits(x, y, h0 + delta * i / n, tight, 0.0)
                   or not footprint_clear_of_unseen_walls(bot, x, y, h0 + delta * i / n) for i in range(n + 1))


def split_tight(bot, x, y, heading, radius):
    """Wall dots near (x, y), split into normal ones and ones already inside the safety margin
    at this pose. The latter only have to not be touched, so the bot can still back or turn
    away from a wall it ended up slightly too close to, instead of freezing."""
    near = dots_near(bot, x, y, radius)
    tight = [p for p in near if footprint_hits(x, y, heading, [p], BODY_MARGIN_IN)]
    return [p for p in near if p not in tight], tight


def turn_steps(angle):
    return round(abs(angle) / (math.pi / 2) * STEPS_PER_90_DEG)


def clear_run(bot, want_in, direction=1, heading=None):
    """How far (inches, up to want_in) the bot can drive straight forward (or back,
    direction=-1) before its footprint would touch a wall dot. `heading` asks the
    same question as if it were already facing another way."""
    x, y = bot["pos_in"]
    h = bot["heading"] if heading is None else heading
    near, tight = split_tight(bot, x, y, h, want_in + REACH_IN + BODY_MARGIN_IN)
    d, step = 0.0, 0.25
    while d + step <= want_in + 1e-9:
        nx = x + direction * (d + step) * math.cos(h)
        ny = y + direction * (d + step) * math.sin(h)
        if (footprint_hits(nx, ny, h, near, BODY_MARGIN_IN) or footprint_hits(nx, ny, h, tight, 0.0)
                or not footprint_clear_of_unseen_walls(bot, nx, ny, h)):
            break
        d += step
    return d


class Autopilot:
    """AUTO EXPLORE: sweep as much floor as the sensors can see, without touching walls.

    The bot pivots near its rear, so turning in place sweeps the body over a disc about
    2 x REACH_IN across. Every time the bot is idle (move done, fresh at-rest readings):
    - uncovered floor within reach of here -> turn through just the arc that sweeps it
    - else drive (turn to face it, then up to MAX_MOVE_IN) towards the cheapest spot
      whose disc still holds uncovered floor: path length + turning - what it would sweep.
      Paths only use cells with room to turn in place, so the bot never gets wedged
    - nothing left: look around (full turn) if it hasn't near here, else done
    Every turn is checked against the swept footprint, every drive against the body's
    path and the live front sensor; mid-drive, two close front readings hard-stop it.
    """

    def __init__(self):
        self.on = False
        self.status = "off"
        self.waiting = False
        self.queue = []           # follow-up commands (second half of a sweep)
        self.path = []            # planned cells (drawn on the map)
        self.blocked = set()      # cells we couldn't reach even though the map said we could
        self.given_up = set()     # floor still uncovered after sweeping at it: out of reach
        self.sweep_cells = set()  # floor the running sweep is meant to cover
        self.backups = 0          # escape attempts in a row that got no closer to open floor
        self.escape_best = math.inf

    def start(self):
        self.on, self.waiting, self.status = True, False, "starting"
        self.queue, self.path = [], []
        self.blocked.clear()
        self.given_up.clear()
        self.sweep_cells.clear()
        self.backups = 0

    def stop(self, why="off"):
        if self.on and self.waiting:
            send_cmd("S")
        self.on, self.waiting, self.status = False, False, why
        self.queue, self.path = [], []

    def tick(self):
        """Called every frame from the pygame loop."""
        if not self.on:
            return
        with bots_lock:
            if not bots:
                self.status = "waiting for a bot"
                return
            bot = next(iter(bots.values()))
            if time.time() - bot["last_seen"] > 2.0:
                self.status = "bot not responding - paused"
                return
            if self.waiting:
                self._watch(bot)
            elif self.queue:
                self._send(bot, *self.queue.pop(0))
            else:
                self._settle_sweep(bot)
                self._plan(bot)

    # -- running a command --
    def _send(self, bot, cmd, steps, status):
        send_cmd(cmd, steps)
        self.status = status
        self.waiting, self.sent_t = True, time.time()
        self.forward = cmd == "F"
        self.motion_at_send = self.seen_motion = bot["motion_count"]
        self.close_front = 0

    def _watch(self, bot):
        started = bot["motion_count"] > self.motion_at_send
        if started and bot["pings_since_motion"] >= 2:
            self.waiting = False            # finished, with fresh at-rest readings
        elif not started and time.time() - self.sent_t > 3.0:
            self.waiting = False            # command lost: plan again
        elif self.forward and bot["motion_count"] != self.seen_motion:
            self.seen_motion = bot["motion_count"]
            f = bot["f"]
            if f < MAX_RANGE_MM and reading_in(f, "f") < STOP_IN:
                self.close_front += 1
                if self.close_front >= 2:
                    send_cmd("S")
                    self.forward = False
                    self.queue = []
                    self.status = "wall close ahead - stopped"
            else:
                self.close_front = 0

    # -- sweeping: turning in place over nearby uncovered floor --
    def _reachable_floor(self, bot, x, y):
        """Uncovered, known-free cells the body can sweep by turning in place with the axle at (x, y)."""
        r = REACH_IN - 0.25
        R = math.ceil(r / CELL_IN) + 1
        c0 = cell_of(x, y)
        out = []
        for dx in range(-R, R + 1):
            for dy in range(-R, R + 1):
                c = (c0[0] + dx, c0[1] + dy)
                if c in bot["free"] and c not in bot["covered"] and c not in self.given_up:
                    cx, cy = cell_center(c)
                    if math.hypot(cx - x, cy - y) <= r:
                        out.append(c)
        return out

    def _settle_sweep(self, bot):
        """Whatever the last sweep didn't cover is out of reach (a wall is in the way): stop chasing it."""
        if self.sweep_cells:
            self.given_up |= {c for c in self.sweep_cells if c not in bot["covered"]}
            self.sweep_cells = set()

    def _sweep(self, bot, cells, label="sweeping"):
        """Turn in place through the smallest arc that sweeps the body over `cells`.
        Returns False if no safe way to do it."""
        cmds = self._sweep_cmds(bot, cells, label)
        if cmds:
            self._do(bot, ("run", cmds, ("sweep", cells)))
        return bool(cmds)

    def _sweep_cmds(self, bot, cells, label="sweeping"):
        """The turn commands for _sweep, or None if there's no safe arc."""
        x, y = bot["pos_in"]
        h = bot["heading"]
        rel = sorted((math.atan2(cy - y, cx - x) - h + math.pi) % (2 * math.pi) - math.pi
                     for cx, cy in map(cell_center, cells))
        if len(rel) == 1:
            start, arc = rel[0], 0.0
        else:
            gaps = [(rel[(i + 1) % len(rel)] - rel[i]) % (2 * math.pi) for i in range(len(rel))]
            i = max(range(len(rel)), key=gaps.__getitem__)       # widest empty gap: don't sweep that
            start, arc = rel[(i + 1) % len(rel)], 2 * math.pi - gaps[i]
        if arc > math.radians(300):
            plans = [[(2 * math.pi, 0.0)]]                          # nearly all round: one full turn
        else:
            end = (start + arc + math.pi) % (2 * math.pi) - math.pi
            plans = sorted([[(start, h), (arc, h + start)], [(end, h), (-arc, h + end)]],
                           key=lambda p: abs(p[0][0]) + abs(p[1][0]))  # turn to one end, sweep to the other
        for plan in plans:
            turns = [(a, h0) for a, h0 in plan if turn_steps(a) > 0]
            if not turns:
                return None       # already facing them: they're just out of the body's reach
            if all(turn_is_clear(bot, a, from_heading=h0) for a, h0 in turns):
                return [("L" if a > 0 else "R", turn_steps(a),
                         f"{label} {len(cells)} sq in ({math.degrees(a):+.0f} deg)") for a, _ in turns]
        return None

    # -- deciding what to do next --
    def _passable(self, bot, clearance=None):
        """Cells the axle may use: seen free, with room to turn in place (PASS_CLEAR_IN from any
        wall dot). Lone dots (no other dot within 1 in) are left out here so a stray noisy reading
        can't cut a route in two; every actual move/turn is still checked against all dots."""
        by_cell = bot["dot_cells"]

        def supported(p):
            c = cell_of(p[0], p[1])
            return any(q != p and abs(q[0] - p[0]) <= 1.0 and abs(q[1] - p[1]) <= 1.0
                       for dx in (-1, 0, 1) for dy in (-1, 0, 1) for q in by_cell.get((c[0] + dx, c[1] + dy), ()))

        clearance = PASS_CLEAR_IN if clearance is None else clearance
        unsafe = set(self.blocked)
        sure, maybe = clearance_offsets(clearance)
        for dc, dots in by_cell.items():          # per wall-dot cell, not per dot: much faster
            real = [q for q in dots if supported(q)]
            if not real:
                continue
            unsafe.update((dc[0] + dx, dc[1] + dy) for dx, dy in sure)
            for dx, dy in maybe:
                c = (dc[0] + dx, dc[1] + dy)
                if c not in unsafe:
                    cx, cy = cell_center(c)
                    if any(math.hypot(cx - px, cy - py) <= clearance for px, py, _ in real):
                        unsafe.add(c)
        free = bot["free"]
        return lambda c: c in free and c not in unsafe

    def _plan(self, bot):
        self._do(bot, self._decide(bot))

    def _do(self, bot, action):
        """Carry out a decision: ("run", commands, tag) / ("escape", why) / ("stop", why) / ("status", text)."""
        kind = action[0]
        if kind == "run":
            cmds, tag = action[1], action[2]
            self.escape_best = math.inf
            if tag and tag[0] == "sweep":
                self.sweep_cells = set(tag[1])
            if tag and tag[0] == "scan":
                bot["scans"].append(bot["pos_in"])
            self.backups = 0
            self._send(bot, *cmds[0])
            self.queue = list(cmds[1:])
        elif kind == "escape":
            self._escape(bot, why=action[1])
        elif kind == "stop":
            self.stop(action[1])
        else:
            self.status = action[1]

    def _decide(self, bot):
        """What AUTO EXPLORE would do next (see the class docstring), without doing it."""
        x, y = bot["pos_in"]
        here = cell_of(x, y)

        nearby = self._reachable_floor(bot, x, y)
        if len(nearby) >= 2:
            cmds = self._sweep_cmds(bot, nearby)
            if cmds:
                return ("run", cmds, ("sweep", nearby))
            self.given_up |= set(nearby)          # can't sweep them safely from here

        # normal margin first; if noise has sealed us off, try once more with no margin
        # (every actual move/turn is still checked against the swept footprint)
        for clearance in (PASS_CLEAR_IN, REACH_IN):
            passable = self._passable(bot, clearance)
            prev, order, frontier = {here: None}, [here], [here]   # BFS from here over passable cells
            while frontier:
                nxt = []
                for c in frontier:
                    for d in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                        n = (c[0] + d[0], c[1] + d[1])
                        if n not in prev and passable(n):
                            prev[n] = c
                            nxt.append(n)
                order.extend(nxt)
                frontier = nxt
            gain = {c: len(self._reachable_floor(bot, *cell_center(c))) for c in order[1:]}
            goals = [c for c in order[1:] if gain[c] >= 2]
            if goals:
                break
        depth = {here: 0}
        for c in order[1:]:
            depth[c] = depth[prev[c]] + 1

        def turn_to(c):
            cx, cy = cell_center(c)
            want = math.atan2(cy - y, cx - x)
            return (want - bot["heading"] + math.pi) % (2 * math.pi) - math.pi
        if not goals:
            self.path = []
            if not any(math.hypot(x - sx, y - sy) < SCAN_SPACING_IN for sx, sy in bot["scans"]):
                if turn_is_clear(bot, 2 * math.pi):
                    return ("run", [("L", 4 * STEPS_PER_90_DEG, "looking around (full turn)")], ("scan",))
                bot["scans"].append((x, y))       # can't turn fully here: don't keep trying
            # boxed into a tight spot while there's still floor to sweep elsewhere: back out first
            if not passable(here) and any(passable(c) and len(self._reachable_floor(bot, *cell_center(c))) >= 2
                                          for c in list(bot["free"])):
                return ("escape", "too close to a wall")
            return ("stop", f"done: covered {len(bot['covered']) * CELL_IN ** 2:.0f} sq in, nothing else reachable")

        goal = min(goals, key=lambda c: (depth[c] * CELL_IN + TURN_COST_IN * abs(turn_to(c)) / math.pi
                                         - SWEEP_VALUE_IN * gain[c]))
        path = [goal]
        while prev[path[-1]] is not None:
            path.append(prev[path[-1]])
        self.path = path[::-1]

        # aim at the furthest path cell the axle can reach in a straight line
        aim = self.path[1] if len(self.path) > 1 else goal
        for c in reversed(self.path[1:]):
            cx, cy = cell_center(c)
            n = max(1, int(math.hypot(cx - x, cy - y) / 0.25))
            cells = (cell_of(x + (cx - x) * i / n, y + (cy - y) * i / n) for i in range(n + 1))
            if all(c2 == here or passable(c2) for c2 in cells):
                aim = c
                break

        delta = turn_to(aim)
        if abs(delta) > math.radians(8):
            for d in (delta, delta - math.copysign(2 * math.pi, delta)):   # short way, else long way
                if turn_is_clear(bot, d):
                    return ("run", [("L" if d > 0 else "R", turn_steps(d), f"turning {math.degrees(d):+.0f} deg")], None)
            return ("escape", "can't turn here")

        ax, ay = cell_center(aim)
        want = min(math.hypot(ax - x, ay - y), MAX_MOVE_IN)
        run = clear_run(bot, want)
        f = bot["f"]
        if f < MAX_RANGE_MM:
            run = min(run, reading_in(f, "f") - FRONT_MARGIN_IN)
        if run < 0.3:
            self.blocked.add(aim)     # map said go, the body/sensor says no: pick something else
            return ("status", "blocked ahead - re-planning")
        return ("run", [("F", round(run * STEPS_PER_INCH), f"forward {run:.1f} in")], None)

    def _escape(self, bot, why="too close to a wall"):
        """Get back to a cell with room to turn: head for the nearest one (through any
        seen-free floor), turning only if the swept body is clear, otherwise creeping
        straight forward or back, whichever gets closer."""
        if self.backups >= 6:
            self.stop(f"stuck ({why}) - drive it out by hand, then AUTO again")
            return
        self.backups += 1
        x, y = bot["pos_in"]
        here, passable, free = cell_of(x, y), self._passable(bot), bot["free"]
        prev, frontier, found = {here: None}, [here], None
        while frontier and not found:
            nxt = []
            for c in frontier:
                for d in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    n = (c[0] + d[0], c[1] + d[1])
                    if n not in prev and n in free:
                        prev[n] = c
                        nxt.append(n)
                        if passable(n):
                            found = n
                            break
                if found:
                    break
            frontier = nxt
        f = bot["f"]
        front_room = reading_in(f, "f") - FRONT_MARGIN_IN if f < MAX_RANGE_MM else 1.0
        if found:
            self.path = [found]
            cx, cy = cell_center(found)
            dist = math.hypot(cx - x, cy - y)
            if dist < self.escape_best - 0.2:      # getting closer: that's progress, not flailing
                self.escape_best = dist
                self.backups = 1
            delta = (math.atan2(cy - y, cx - x) - bot["heading"] + math.pi) % (2 * math.pi) - math.pi
            if abs(delta) > math.radians(8) and abs(delta) < math.radians(172):
                for d in (delta, delta - math.copysign(2 * math.pi, delta)):
                    if turn_is_clear(bot, d):
                        self._send(bot, "L" if d > 0 else "R", turn_steps(d),
                                   f"{why} - turning {math.degrees(d):+.0f} deg to get out")
                        return
            # can't (or needn't) turn: go straight whichever way gets closer
            direction = 1 if abs(delta) < math.pi / 2 else -1
            want = min(max(dist * abs(math.cos(delta)), 0.5), 1.5)
        else:
            direction, want = -1, 1.0
        for dirn in (direction, -direction):
            run = clear_run(bot, want, direction=dirn)
            if dirn == 1:
                run = min(run, front_room)
            if run >= 0.3:
                self._send(bot, "F" if dirn == 1 else "B", round(run * STEPS_PER_INCH),
                           f"{why} - {'creeping forward' if dirn == 1 else 'backing up'} {run:.1f} in")
                return
        self.stop(f"stuck ({why}) - drive it out by hand, then AUTO again")


autopilot = Autopilot()


# ---------------- Rendering ----------------
def to_screen(x_in, y_in):
    # mat is y-up (CCW-positive theta); pygame is y-down, so flip y
    # round: pygame truncates, so 474.9999 vs 475.0001 would land a pixel apart
    return (round(VIEW_CX + (x_in - START_IN[0]) * px_per_in),
            round(VIEW_CY - (y_in - START_IN[1]) * px_per_in))


def zoom(factor):
    global px_per_in
    px_per_in = min(max(px_per_in * factor, ZOOM_RANGE[0]), ZOOM_RANGE[1])


def nice_step(min_px, steps=(1, 2, 5, 10, 20, 50, 100)):
    """Smallest step in inches that spans at least min_px on screen at the current zoom."""
    return next((s for s in steps if s * px_per_in >= min_px), steps[-1])


def fmt_in(bot, key):
    return "--" if bot[key] >= MAX_RANGE_MM else f"{reading_in(bot[key], key):.1f}in"


def draw_grid(screen, label_font):
    """Inch grid matching the mat's markings, filling the view (which is centred on
    START_IN). Lines and labels thin out as you zoom out; the mat's 0 lines are brighter."""
    step, lstep = nice_step(8), nice_step(30)   # grid line / label spacing, inches
    x_lo = math.floor(START_IN[0] - (VIEW_CX - PANEL_W) / px_per_in)
    x_hi = math.ceil(START_IN[0] + (WINDOW_W - VIEW_CX) / px_per_in)
    y_lo = math.floor(START_IN[1] - (WINDOW_H - VIEW_CY) / px_per_in)
    y_hi = math.ceil(START_IN[1] + VIEW_CY / px_per_in)

    def multiples(lo, hi, s):
        return range(math.ceil(lo / s) * s, hi + 1, s)

    for i in multiples(x_lo, x_hi, step):
        sx, _ = to_screen(i, 0)
        pygame.draw.line(screen, (110, 110, 110) if i == 0 else (45, 45, 45), (sx, 0), (sx, WINDOW_H))
    for j in multiples(y_lo, y_hi, step):
        _, sy = to_screen(0, j)
        pygame.draw.line(screen, (110, 110, 110) if j == 0 else (45, 45, 45), (PANEL_W, sy), (WINDOW_W, sy))
    for i in multiples(x_lo, x_hi, lstep):
        sx, _ = to_screen(i, 0)
        lbl = label_font.render(str(i), True, (150, 150, 150), (0, 0, 0))
        lx = sx - lbl.get_width() // 2
        if PANEL_W + 30 <= lx and lx + lbl.get_width() <= WINDOW_W - 2:  # clear of the y labels / edge
            screen.blit(lbl, (lx, WINDOW_H - lbl.get_height() - 4))
    for j in multiples(y_lo, y_hi, lstep):
        _, sy = to_screen(0, j)
        if 0 <= sy - 6 and sy + 6 <= WINDOW_H - 20:  # on screen, clear of the x labels
            lbl = label_font.render(str(j), True, (150, 150, 150), (0, 0, 0))
            screen.blit(lbl, (PANEL_W + 4, sy - lbl.get_height() // 2))


def draw_scale(screen, label_font):
    """Scale bars in inches and cm, top-right of the view; their length follows the zoom."""
    x0, y = WINDOW_W - 155, 24
    pygame.draw.rect(screen, (15, 15, 15), (x0 - 12, 8, 160, 60))
    pygame.draw.rect(screen, (90, 90, 90), (x0 - 12, 8, 160, 60), 1)
    for units, px_per_unit in (("in", px_per_in), ("cm", px_per_in / 2.54)):
        # longest round length that fits in ~100 px
        n = max([c for c in (1, 2, 5, 10, 20, 50, 100, 200) if c * px_per_unit <= 100] or [1])
        ticks = n if n <= 10 else 5
        pygame.draw.line(screen, (220, 220, 220), (x0, y), (x0 + n * px_per_unit, y), 2)
        for k in range(ticks + 1):
            tick = 6 if k in (0, ticks) else 3
            sx = x0 + k * n * px_per_unit / ticks
            pygame.draw.line(screen, (220, 220, 220), (sx, y - tick), (sx, y + tick))
        lbl = label_font.render(f"{n} {units}", True, (220, 220, 220))
        screen.blit(lbl, (x0 + n * px_per_unit + 8, y - lbl.get_height() // 2))
        y += 28

    # colour key for the sensor dots, just under the scale bars
    pygame.draw.rect(screen, (15, 15, 15), (x0 - 12, 72, 160, 60))
    pygame.draw.rect(screen, (90, 90, 90), (x0 - 12, 72, 160, 60), 1)
    for i, key in enumerate(("f", "l", "r")):
        cy = 86 + i * 16
        pygame.draw.circle(screen, SENSOR_COLORS[key], (x0 + 4, cy), 4)
        lbl = label_font.render(f"{SENSOR_NAMES[key]} sensor", True, SENSOR_COLORS[key])
        screen.blit(lbl, (x0 + 14, cy - lbl.get_height() // 2))


def draw_panel(screen, font, small_font):
    pygame.draw.rect(screen, (25, 25, 30), (0, 0, PANEL_W, WINDOW_H))
    y = 15

    with bots_lock:
        bot_items = list(bots.items())

    def line(text, color=(230, 230, 230), bold_font=None):
        nonlocal y
        surf = (bold_font or font).render(text, True, color)
        screen.blit(surf, (15, y))
        y += surf.get_height() + 4

    line(f"Bots connected: {len(bot_items)}", (255, 255, 255), font)
    y += 6

    if not bot_items:
        line("Waiting for bot...", (150, 150, 150))

    for bot_id, bot in bot_items:
        line(f"id: {bot_id}", (255, 220, 120))
        line(f"  body: {BOT_WIDTH_IN} x {BOT_LENGTH_IN} in (W x L)", (255, 170, 170), small_font)
        dims = bot.get("dims", {})
        if dims:
            line(f"  wheel_d: {dims.get('wheel_diameter_mm','?')}mm", (180, 180, 180), small_font)
            line(f"  wheel_base: {dims.get('wheel_base_mm','?')}mm", (180, 180, 180), small_font)
            line(f"  steps/rev: {dims.get('steps_per_rev','?')}", (180, 180, 180), small_font)
        else:
            line("  (no handshake yet)", (140, 140, 140), small_font)

        x_in, y_in = bot["pos_in"]
        line(f"  pos: ({x_in:.2f}, {y_in:.2f}) in", (200, 200, 255), small_font)
        line(f"  heading: {heading_deg(bot['heading']):.1f} deg", (200, 200, 255), small_font)
        x_off = 15
        for key, prefix in (("f", "  "), ("r", "  "), ("l", "  ")):
            surf = small_font.render(f"{prefix}{key.upper()}={fmt_in(bot, key)}", True, SENSOR_COLORS[key])
            screen.blit(surf, (x_off, y))
            x_off += surf.get_width()
        y += surf.get_height() + 4
        line(f"  bat={bot.get('bat')}", (200, 200, 200), small_font)

        if SHOW_BOX and box_visible:
            # mean (reading - expected) per sensor since reset; subtracting it from
            # that sensor's offset would zero it
            line("  box error (read - expected):", (120, 220, 120), small_font)
            fixes = []
            for key in ("f", "r", "l"):
                n, total, sq = bot["box_err"][key]
                if n:
                    mean, rms = total / n, math.sqrt(sq / n)
                    line(f"   {key.upper()}: {mean:+.2f}in rms {rms:.2f} n={n}", SENSOR_COLORS[key], small_font)
                    fixes.append(f"{key}{READING_OFFSET_IN[key] - mean:+.2f}")
                else:
                    line(f"   {key.upper()}: no hits inside box", (140, 140, 140), small_font)
            if fixes:
                line("  offset fix: " + " ".join(fixes), (120, 220, 120), small_font)

        last_cmd = bot.get("last_cmd")
        if last_cmd:
            line(f"  last: cmd={last_cmd} dir={bot.get('last_dir')}", (255, 200, 150), small_font)

        age = time.time() - bot["last_seen"]
        stale = age > 2.0
        line(f"  last seen: {age:.1f}s ago", (255, 100, 100) if stale else (150, 150, 150), small_font)
        y += 10

    mouse = pygame.mouse.get_pos()

    def button(rect, text, color, hover_color):
        pygame.draw.rect(screen, hover_color if rect.collidepoint(mouse) else color, rect, border_radius=6)
        lbl = small_font.render(text, True, (255, 255, 255))
        screen.blit(lbl, lbl.get_rect(center=rect.center))

    grey, grey_hi = (60, 60, 75), (90, 90, 110)
    button(AUTO_BTN, "AUTO: ON (stop)" if autopilot.on else "AUTO EXPLORE",
           (160, 100, 20) if autopilot.on else (75, 62, 30), (195, 130, 40))
    button(ZOOM_OUT_BTN, "ZOOM -", grey, grey_hi)
    button(ZOOM_IN_BTN, "ZOOM +", grey, grey_hi)
    button(FULL_BTN, "WINDOW" if fullscreen else "FULLSCREEN", grey, grey_hi)
    if SHOW_BOX:
        button(BOX_BTN, "HIDE BOX" if box_visible else "SHOW BOX", (25, 105, 50), (40, 150, 75))
    button(SPIN_BTN, "ROTATE 360", (30, 70, 140), (50, 100, 185))
    button(RESET_BTN, "RESET  (clear map, bot -> start)", (130, 35, 35), (175, 50, 50))

    y = WINDOW_H - 192
    line("Controls:", (255, 255, 255), font)
    for txt in [
        "W/S or Up/Down: forward / back",
        "A/D or Left/Right: rotate L / R",
        f"Q/E: turn 90 ({STEPS_PER_90_DEG} steps)",
        "[ / ]: steps per 90 -/+ 10",
        f"+/-: step {step_size} ({step_size / STEPS_PER_INCH:.2f} in)",
        "SPACE: HARD STOP",
        "Wheel: zoom   Esc: exit full screen",
    ]:
        line(txt, (170, 170, 170), small_font)


def draw_pilot_status(screen, text_font, bot_items):
    """Top-left of the map: what AUTO EXPLORE is doing (for demos, too)."""
    lines = []
    if autopilot.on or autopilot.status != "off":
        for i, chunk in enumerate(textwrap.wrap(f"AUTO EXPLORE: {autopilot.status}", 58)):
            lines.append((("   " if i else "") + chunk, (240, 170, 60)))
        if bot_items:
            covered = len(bot_items[0][1]["covered"]) * CELL_IN ** 2
            lines.append((f"ground covered: {covered:.0f} sq in", (90, 200, 200)))
    if not lines:
        return
    surfs = [text_font.render(t, True, c) for t, c in lines]
    w = max(s.get_width() for s in surfs) + 16
    h = sum(s.get_height() + 2 for s in surfs) + 10
    box = pygame.Rect(PANEL_W + 36, 8, w, h)
    pygame.draw.rect(screen, (12, 12, 16), box)
    pygame.draw.rect(screen, (90, 90, 90), box, 1)
    y = box.y + 5
    for s in surfs:
        screen.blit(s, (box.x + 8, y))
        y += s.get_height() + 2


def draw_world(screen, label_font, text_font=None):
    draw_grid(screen, label_font)

    with bots_lock:
        bot_items = list(bots.items())
        covered = [list(bot["covered"]) for _, bot in bot_items]   # the MQTT thread keeps changing these
        dots = [list(bot["points"]) for _, bot in bot_items]

    # ground the body has been over
    size = max(1, round(CELL_IN * px_per_in))
    for cells in covered:
        for (cx, cy) in cells:
            sx, sy = to_screen(cx * CELL_IN, (cy + 1) * CELL_IN)
            if PANEL_W - size < sx < WINDOW_W and -size < sy < WINDOW_H:
                pygame.draw.rect(screen, (0, 52, 58), (sx, sy, size, size))

    # the autopilot's planned route
    if autopilot.on and autopilot.path:
        for c in autopilot.path[1:]:
            pygame.draw.circle(screen, (240, 170, 60), to_screen(*cell_center(c)), 2)
        pygame.draw.circle(screen, (240, 170, 60), to_screen(*cell_center(autopilot.path[-1])), 6, 2)

    # calibration box: where the walls should be (green) vs what the sensors see (dots)
    if SHOW_BOX and box_visible:
        x0, y0, x1, y1 = box_rect()
        corners = [to_screen(x0, y0), to_screen(x1, y0), to_screen(x1, y1), to_screen(x0, y1)]
        pygame.draw.polygon(screen, (0, 200, 70), corners, 2)

    # point cloud
    for bot_dots in dots:
        for (px, py, key) in bot_dots:
            sx, sy = to_screen(px, py)
            if PANEL_W < sx < WINDOW_W and 0 < sy < WINDOW_H:
                pygame.draw.circle(screen, SENSOR_COLORS[key], (int(sx), int(sy)), 3)

    # bots + live sensor rays
    for bot_id, bot in bot_items:
        x_in, y_in = bot["pos_in"]
        theta = bot["heading"]

        def body_pt(fwd, left):
            return to_screen(*body_to_mat(x_in, y_in, theta, fwd, left))

        # footprint, to scale
        rear = AXLE_TO_FRONT_IN - BOT_LENGTH_IN
        half_w = BOT_WIDTH_IN / 2
        corners = [body_pt(AXLE_TO_FRONT_IN, half_w), body_pt(AXLE_TO_FRONT_IN, -half_w),
                   body_pt(rear, -half_w), body_pt(rear, half_w)]
        pygame.draw.polygon(screen, (80, 20, 20), corners)
        pygame.draw.polygon(screen, (230, 60, 60), corners, 2)

        # drive wheels on the axle
        dims = bot["dims"]
        half_base = WHEEL_TRACK_IN / 2
        wheel_r = dims.get("wheel_diameter_mm", FW_WHEEL_DIAMETER_MM) / MM_PER_INCH / 2
        for side in (1, -1):
            pygame.draw.line(screen, (190, 190, 190),
                             body_pt(-wheel_r, side * half_base), body_pt(wheel_r, side * half_base), 6)

        # axle midpoint (the tracked point) + heading to the front edge
        bx, by = to_screen(x_in, y_in)
        pygame.draw.line(screen, (255, 90, 90), (bx, by), body_pt(AXLE_TO_FRONT_IN, 0), 3)
        pygame.draw.circle(screen, (255, 90, 90), (int(bx), int(by)), 5)

        # live sensor rays, each starting from its sensor
        for key in SENSOR_MOUNTS:
            sx, sy, beam = sensor_ray(bot, key)
            dist_in = reading_in(bot[key], key)
            start = to_screen(sx, sy)
            end = to_screen(sx + dist_in * math.cos(beam), sy + dist_in * math.sin(beam))
            pygame.draw.line(screen, SENSOR_COLORS[key], start, end, 2)
            pygame.draw.circle(screen, SENSOR_COLORS[key], (int(start[0]), int(start[1])), 4)

    draw_scale(screen, label_font)
    draw_pilot_status(screen, text_font or label_font, bot_items)


def set_fullscreen(on):
    """Full screen at the display's own resolution, or back to the normal window."""
    global fullscreen
    if fullscreen and not on:
        # set_mode alone can keep the full-screen size, so rebuild the window
        pygame.display.quit()
        pygame.display.init()
    fullscreen = on
    screen = (pygame.display.set_mode((0, 0), pygame.FULLSCREEN) if on
              else pygame.display.set_mode(WINDOWED_SIZE))
    pygame.display.set_caption("Nanodot — Live Simulation")
    set_layout(*screen.get_size())
    return screen


def take_over(why="manual control"):
    """Stop the autopilot if it's driving (it hard-stops the bot if a move is running)."""
    if autopilot.on:
        autopilot.stop(why)


def run_pygame():
    global step_size, STEPS_PER_90_DEG, box_visible
    pygame.init()
    screen = set_fullscreen(False)
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("consolas", 18)
    small_font = pygame.font.SysFont("consolas", 14)
    label_font = pygame.font.SysFont("consolas", 12)

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                if RESET_BTN.collidepoint(event.pos):
                    take_over("reset")
                    reset_all()
                elif AUTO_BTN.collidepoint(event.pos):
                    if autopilot.on:
                        autopilot.stop("stopped")
                    else:
                        autopilot.start()
                elif SHOW_BOX and BOX_BTN.collidepoint(event.pos):
                    box_visible = not box_visible
                elif SPIN_BTN.collidepoint(event.pos):
                    take_over()
                    send_cmd("L", 4 * STEPS_PER_90_DEG)   # one full turn, as a single move
                elif ZOOM_IN_BTN.collidepoint(event.pos):
                    zoom(1.25)
                elif ZOOM_OUT_BTN.collidepoint(event.pos):
                    zoom(1 / 1.25)
                elif FULL_BTN.collidepoint(event.pos):
                    screen = set_fullscreen(not fullscreen)
            elif event.type == pygame.MOUSEWHEEL:
                zoom(1.1 ** event.y)
            elif event.type == pygame.KEYDOWN:
                if event.key in (pygame.K_w, pygame.K_UP, pygame.K_s, pygame.K_DOWN, pygame.K_a, pygame.K_LEFT,
                                 pygame.K_d, pygame.K_RIGHT, pygame.K_q, pygame.K_e, pygame.K_SPACE):
                    take_over()   # any driving key hands control back to you
                if event.key == pygame.K_ESCAPE and fullscreen:
                    screen = set_fullscreen(False)
                elif event.key in (pygame.K_w, pygame.K_UP):
                    send_cmd("F", step_size)
                elif event.key in (pygame.K_s, pygame.K_DOWN):
                    send_cmd("B", step_size)
                elif event.key in (pygame.K_a, pygame.K_LEFT):
                    send_cmd("L", step_size)
                elif event.key in (pygame.K_d, pygame.K_RIGHT):
                    send_cmd("R", step_size)
                elif event.key == pygame.K_q:
                    send_cmd("L", STEPS_PER_90_DEG)
                elif event.key == pygame.K_e:
                    send_cmd("R", STEPS_PER_90_DEG)
                elif event.key == pygame.K_SPACE:
                    send_cmd("S")
                elif event.key in (pygame.K_PLUS, pygame.K_EQUALS):
                    step_size = min(step_size + 50, 2000)
                elif event.key == pygame.K_MINUS:
                    step_size = max(step_size - 50, 50)
                elif event.key in (pygame.K_LEFTBRACKET, pygame.K_RIGHTBRACKET):
                    STEPS_PER_90_DEG += 10 if event.key == pygame.K_RIGHTBRACKET else -10
                    track = STEPS_PER_90_DEG * 4 / (math.pi * STEPS_PER_INCH)
                    print(f"STEPS_PER_90_DEG = {STEPS_PER_90_DEG}  -> to keep it, set WHEEL_TRACK_IN = {track:.3f} in main.py")

        autopilot.tick()

        screen.fill((0, 0, 0))
        draw_world(screen, label_font, small_font)
        draw_panel(screen, font, small_font)

        pygame.display.flip()
        clock.tick(30)

    pygame.quit()


if __name__ == "__main__":
    t = threading.Thread(target=mqtt_thread, daemon=True)
    t.start()
    run_pygame()