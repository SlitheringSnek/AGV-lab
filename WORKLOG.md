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
| Rear axle → swivel joint | – | **D = 0.12 m (guess, TODO: measure)** |

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
1. Measure `D` (rear axle → swivel joint) and set `joint_dist` in `agv` and `control_point_odom`.
2. Re-test odometry with the corrected model (straight run, curves, RViz against the cyan model).
3. Test line following: check `sign`, tune `kp`/`kd`.
4. Test `control_point_odom`.
5. Optional: correct the wheel-diameter mismatch (UMBmark).
