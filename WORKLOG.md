# AMS lab – work log

Robot: mAGV `klemen` (10.8.134.125), ROS 2 Lyrical, namespace `/klemen`, `ROS_DOMAIN_ID=26`.
Code: `~/ros/src/amsagv` (package built with `colcon build --symlink-install`, so Python edits
take effect without rebuilding).

---

## 2026-11-03 – Task 1 (odometry) and start of Task 2 (control)

### 1. Reading the encoders

**What:** In `agv.py`, the 50 Hz timer now prints the wheel encoders `L`, `R`, the cart encoder `H`,
and the change since the previous print, five times per second. The print can be switched
off with `-p debug:=false`.

**Why:** Before writing any odometry we needed to see the raw data: which way each encoder counts,
how many ticks a metre is, and what `H` measures.

### 2. Manual driving with the keyboard

```bash
# robot
ros2 run amsagv agv
# PC
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.1 -p turn:=1.0
```

Findings:
- `i` = forward (towards the front cart), `,` = back, `k` = stop. `m` is *back + turn*, not straight back.
- **The left encoder counts down when driving forward, the right one counts up.**
- The robot stops shortly after the key is released (watchdog in the motor driver).
- When reversing, the front cart can fold round 180°. After that the robot "does whatever it
  wants". Straighten the cart by hand.

### 3. Calibration measurements

| Quantity | Measurement | Result |
|---|---|---|
| Wheel ticks per metre | clean straight run, 0.50 m measured: ΔL = −55640, ΔR = +55565 | **K = 111 200 ticks/m** |
| Cart encoder resolution | cart turned ±90° by hand: ΔH ≈ +2040 / −2068 | **8192 ticks/turn** (13-bit), 1303.8 ticks/rad |
| Cart straight (zero) | `H` with cart straight | **4660**; `H` wraps every 8192, the value repeats after a full turn |
| Distance between driven wheels | 360° cart rotation: dsR − dsL = 0.306 m → 0.306/2π; ruler: 49 mm centre to centre | **L = 0.0487 m** |
| Rear axle → swivel joint | ruler, rear axle to swivel joint (matches the 120.7 mm in `magv.pdf`) | **D = 0.1207 m** |

Notes:
- An earlier 70 cm run gave ~119 000 ticks/m, but that run ended with a disturbance, so it was not used.
- `agvapi.py` uses `AXIS_LENGTH = 0.075` in `set_vel()`. The real value is 0.049, so the robot turns
  about 1.5× faster than the commanded `w`. This does not affect odometry. The lab file was left unchanged.
- Small drift to the right on straight runs (~2 cm per 50 cm) that the encoders don't see.
  Probably the two wheels differ slightly in diameter (a systematic odometry error).

### 4. Odometry (in `agv.py`)

**Kinematic model** (from the diagram in the Task 2 slides): the **driven wheels are on the front
cart** (differential drive). The rear wheels of the body are passive. The cart is attached with a
swivel joint at distance `D` in front of the rear axle. `gamma` is the cart angle relative to the
body and is measured directly by the `H` encoder.

```
dsL = -dL / K,  dsR = dR / K               wheel displacements (left encoder inverted)
gamma = wrap((H - 4660) / 1303.8)          cart angle, + = left
ds_s  = (dsR + dsL) / 2                    cart travel
fd   += ds_s                               travelled distance of the cart (line.distance)
ds    = ds_s * cos(gamma)                  rear-axle travel
dphi  = ds_s * sin(gamma) / D              body rotation
x += ds * cos(phi + dphi/2)                2nd-order Runge-Kutta (midpoint) integration
y += ds * sin(phi + dphi/2)
phi = wrap(phi + dphi)
```

Published on `/klemen/odom` (`pose = (x, y, phi)`, `position.z = gamma`, as the template expects) and
`/klemen/line` (`heading = gamma`, `distance = fd`).

ROS parameters: `ticks_per_m`, `heading_offset`, `joint_dist`, `debug`.

**Mistake and correction:** the first version assumed the driven wheels were on the body, with a
passive cart. That model was used for the RViz tests below. It explains why turning in place
(`j`/`l`) produced a fan of arrows without the body turning: in reality only the cart was spinning.
The model was corrected after reading the Task 2 slides. **The corrected model has not been tested
on the robot yet.**

### 5. Verification in RViz

```bash
# PC, separate terminals
ros2 run amsagv set_odom_frame
ros2 launch amsagv rviz.launch.xml
ros2 run teleop_twist_keyboard teleop_twist_keyboard ...
```

- Use the *2D Pose Estimate* tool on top of the robot's own coloured model (ours = **cyan**).
  The coloured models are the robots' real positions from the lab's camera system.
- Order: place robot → restart `agv` (odometry = 0) → clear arrows → 2D Pose Estimate → drive.
- The odometry arrows roughly followed the robot. Most error came from turning in place and from
  placing the initial pose.
- **Gotcha:** arrows disappeared completely once. The cause was `position.z = gamma`: with the cart
  folded (`gamma` = −2.49 rad), RViz draws the arrows 2.5 m below the floor. Straighten the cart.
- Arrows appear in bursts because the Odometry display only adds one per 5 cm / 0.1 rad
  (Position/Angle Tolerance).

### 6. Task 2 – first implementation (NOT TESTED YET)

**`control_line.py` – line following**
- Error `e = sign * (left + right) / 2` (line position under the sensor, 0 = centre).
- PD control `w = kp*e + kd*de`, speed `v = v0 * max(0.3, 1 - |e|)` (slower in sharp turns).
- Line lost: keep the last error at half speed for `lost_max` samples (0.3 s), then stop.
- Parameters: `v` (0.1), `kp` (2.0), `kd` (0.0), `sign` (1.0, set to −1 if it steers away), `lost_max`.
- Tags: `tag_reader` (robot) + `tag_beep` (PC) beep on every NFC tag.

**`control_point_odom.py` – drive to a point using odometry**
- Controls the cart, because that's where the driven wheels are: joint position
  `(x + D cos phi, y + D sin phi)` and heading `phi + gamma`.
- `ws = kw * alpha`, `vs = kv * dist * max(0, cos(alpha))`, saturated to `v_max` / `w_max`.
  Stops within `tol` (2 cm).
- Target in the odom frame: `-p x_ref:=0.4 -p y_ref:=0.2`.

### Open items / next session
1. Re-test odometry with the corrected model (straight run, curves, RViz against the cyan model).
2. Test line following: check `sign`, tune `kp`/`kd`.
3. Test `control_point_odom`.
4. Optional: correct the wheel-diameter mismatch (UMBmark).

---

## 2026-11-10 – Simulation test, first real line-following runs, crossroads

### 1. `vaje-pdf/amsagv-new_code` vs our code

The new folder is the **unfilled lab template**. 20 files are byte-identical to ours. The 3 that
differ (`agv.py`, `control_line.py`, `control_point_odom.py`) still have `#TODO` / `v, w = 0, 0`
stubs there; ours are supersets of them. **Nothing to integrate.**

### 2. Simulation test (no hardware)

`/tmp/agvsim/sim_agv.py` replaces `Agv` with a simulated robot (cart kinematics, measured K, L, D,
heading offset/wrap, the real `set_vel()` mapping incl. `AXIS_LENGTH = 0.075`, 0.3 s watchdog) and
runs our **unchanged** `agv.py`. `run.sh point|line <s> [params]` starts it plus the controller on
ROS domain 99, localhost-only, namespace `/simtest`. `/tmp` is wiped on reboot.

Results:
- **Odometry:** estimate = simulated truth to the mm in all runs (signs, K, kinematics consistent).
  Does NOT verify the real `gamma` sign.
- **Line following** (assumed sensor geometry, 3 cm ahead of the wheels): 1 cm start offset corrected
  in ~1 s, 0.5 m arc tracked with 2.6 mm offset, no oscillation, with the default gains.
- **Point control:** all goals reached, but a goal behind the robot folded the cart to **−152°**.

### 3. Fix: cart-angle limit in `control_point_odom.py`

If |gamma| > `gamma_max` and the controller wants to turn the cart further, creep forward at
`v_turn` and hold gamma with `ws = vs*sin(gamma)/D - kg*(excess)`. Parameters: `gamma_max` (70.0 deg),
`v_turn` (0.05), `kg` (3.0). After the fix, max |gamma| was 75° in all 6 test goals (the extra ~5° is the
1.54× turn-rate error from `AXIS_LENGTH`). The debug print now also shows gamma.

Notes:
- The robot always stops ~2 cm short: it stops as soon as it is inside `tol`. Lower `tol` if needed.
- The `RCLError: ... context is invalid` traceback on Ctrl-C (or SIGTERM) is a shutdown race. It is
  harmless: the motors are still stopped by `Agv.__exit__`.
- **Not tested on the robot yet.**

### 4. Gotcha: ROS parameters need `--ros-args`

`ros2 run amsagv control_line -p v:=0.0` **silently ignores** the parameter. Correct:

```bash
ros2 run amsagv control_line --ros-args -p v:=0.0 -p kp:=1.5
```

Float parameters must be written as floats (`kp:=2.0`, not `2`), or ROS rejects the type.
The `-p x_ref:=…` example in the 2026-11-03 entry needs `--ros-args` too.

### 5. Line sensor on the real robot

Raw values with the robot standing on the line (read directly with `Agv`, motors not driven):

```
[47, 25, 55, 163, 184, 147, 37]   → tape BRIGHT (~160–190), floor DARK (~25–55)
```

- 7 sensors, **index 0 = right, index 6 = left** (confirmed: `sign = +1` steers the right way).
- Tape width ≈ 2.9 sensor spacings (edges at 2.47 and 5.38) → `half_width = 1.45`.
- The lab's `find_line_edges()` returns exactly `left = 1.0, right = -1.0` when **all** sensors are bright
  (no contrast, e.g. a crossing), and NaN when all are dark. Its edge formulas are slightly asymmetric
  (≈ −0.1 bias with a centred line), so we no longer use it for control.

### 6. First line-following runs and fixes (`control_line.py`)

**First run** (old code; the `-p v:=0.05 -p kp:=2.0` was ignored, so it ran with v=0.1, kp=2.0):
the cart jerked sharply left/right, ran off the line and sometimes the robot drove **backwards**.

Causes:
- On line loss it kept the last error at 15 % speed but full `w` for 0.3 s → cart spun in place, and
  the sensor (in front of the driven wheels) swept back and forth across the line.
- No cart-angle limit → cart folded past 90° → the cart pushes the body backwards.
- The all-bright `(1, -1)` reading gave e = 0 and it drove straight on.

Fixes:
- **Stops immediately** when there is no line.
- Line position = **centroid** of the raw values above the mid threshold, error `e = (pos − 3)/3`
  (+1 = line on the left).
- `w` saturated to `w_max` (1.5 rad/s); `v = v0 * max(0.5, 1 − |e|)` so it keeps moving forward.
- Cart limit: no further turning beyond `gamma_max` (60°); full stop beyond `gamma_stop` (100°).
- Defaults now `v = 0.05`, `kp = 1.5`, `kd = 0`, `sign = +1`.

**Second runs** (all effectively with the defaults, see section 4): **follows the line well**, but at
crossroads it either stopped (all sensors bright) or picked a random branch (the centroid of two
branches lies between them).

### 7. Crossroads: edge following (`control_line.py`)

- `analyse_line(values)` returns the state (`line` / `crossing` = all bright / `no line`), the centroid,
  the outer **left edge** (6.5 = beyond the sensor) and the outer **right edge** (−0.5).
- New parameter `mode`: `center` | `left` | `right`. In `left` mode the left edge is held at
  `3 + half_width`, so at a fork the robot takes the **left** branch; `right` mode the right branch.
  (This is what `line.left` / `line.right` in the message and the template action `('left', 20, 0.202)`
  are meant for.)
- All sensors bright (crossing): drive straight across for at most `cross_dist` (0.06 m), then stop.
- Subscribes to `path_actions` (for the later path-planning task): action `(name, tag_id, distance)`
  switches the mode to `name` (`left` / `right` / `straight` / `stop`) after `distance` m past tag `tag_id`.
- Debug print: 7 raw values, mode, e, gamma, fd, v, w, state.
- Checked on the robot without driving: compiles, `analyse_line` gives the expected results for
  centred / Y-split / crossing / floor readings, `--ros-args -p mode:=left` is picked up.
  **Not yet driven over a crossroads.**

Parameters of `control_line`: `v` (0.05), `kp` (1.5), `kd` (0.0), `w_max` (1.5), `sign` (1.0),
`gamma_max` (60.0), `gamma_stop` (100.0), `mode` (center), `half_width` (1.45), `cross_dist` (0.06), `debug`.

### Status

| Item | State |
|---|---|
| Odometry (corrected cart model) | simulation OK, real robot not re-tested |
| Line following, plain line | **works on the robot** (v = 0.05, kp = 1.5) |
| Stop when off the line | **works** |
| Crossroads (`mode` left/right/center) | implemented, not driven yet |
| `control_point_odom` + cart limit | simulation OK, real robot not tested |
| NFC tags (`tag_reader` + `tag_beep`) | not tested today |

### Open items / next session
1. Drive crossroads with `--ros-args -p mode:=left`, `mode:=right`, `mode:=center`. If it stops in a
   crossing, raise `cross_dist`; if it wobbles in edge mode, lower `kp`.
2. Raise `v` to 0.1 → 0.15, retune `kp` / `kd`.
3. Tags: `tag_reader` (robot) + `tag_beep` (PC), check every tag beeps and `control_line` prints it.
4. Cart turned **left** by hand → `gamma` must be **positive** (odometry and both cart limits rely on it).
5. `control_point_odom --ros-args -p x_ref:=0.5`, then `0.3 0.3`, then a goal behind (`-0.3 0.0`).
6. Later task: implement `planning.py` (`find_path`, `generate_actions`), then `control_line` follows
   `path_actions` automatically.
