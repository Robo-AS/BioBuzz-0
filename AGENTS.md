# BioBuzz FTC Robotics — Agent Instructions & Knowledge Base

This is the central operating context for AI coding assistants working on the **BioBuzz** FIRST Tech Challenge (FTC) robot codebase.

---

## 1. Documentation Map & Sub-Modules

To keep context windows focused and token-efficient, detailed specifications are split into dedicated documents:

- **Competition Rules & Scoring**: [`docs/GAME_RULES.md`](docs/GAME_RULES.md)  
  *Match timing (Auto 30s, TeleOp 2:00, Flower window at 1:00), official scoring table, HIVE tipping (20 pts), FLOWER bonus, and critical penalties to avoid.*
- **Official Q&A Rulings & Edge Cases**: [`docs/QA_RULINGS.md`](docs/QA_RULINGS.md)  
  *GDC interpretations on LEAVE buzzer evaluation, shot-blocking legality, intake 4-ball limit enforcement, FLOWER de-scoring bans, and hardware expansion inspection rules.*
- **Official Manuals (Reference)**:
  - HTML source: [`docs/reference/BIOBUZZ Competition Manual - TU10.htm`](docs/reference/BIOBUZZ%20Competition%20Manual%20-%20TU10.htm)
  - Official PDF: [`docs/reference/BIOBUZZ_Competition_Manual_TU10.pdf`](docs/reference/BIOBUZZ_Competition_Manual_TU10.pdf)
  - Official Q&A Source: [`docs/reference/Team Q&A - FIRST Game Q&A.html`](docs/reference/Team%20Q%26A%20-%20FIRST%20Game%20Q%26A.html)

---

## 2. Project & Hardware Specifications

- **Team**: BioBuzz
- **Season**: FTC 2026–2027 (BIOBUZZ presented by RTX)
- **Control Hardware**: REV Control Hub + Expansion Hub, GoBilda Pinpoint Odometry Computer, Limelight 3A / Webcams, Blinkin LED Driver.
- **Software Stack**: FTC Android SDK, FTCLib 2.1.1 (Command-Based), Pedro Pathing (Foresight algorithm).

### Hardware Map Reference
All hardware device names in code must match these exact strings configured on the Robot Controller:

| Subsystem | Hardware Device | Config Name | Class | Configuration |
| :--- | :--- | :--- | :--- | :--- |
| **Drivetrain** | Front Left Motor | `"frontLeft"` | `DcMotorEx` | Forward (Drive) / Reverse (Pedro) |
| **Drivetrain** | Front Right Motor | `"frontRight"` | `DcMotorEx` | Reverse (Drive) / Forward (Pedro) |
| **Drivetrain** | Rear Left Motor | `"rearLeft"` | `DcMotorEx` | Forward (Drive) / Reverse (Pedro) |
| **Drivetrain** | Rear Right Motor | `"rearRight"` | `DcMotorEx` | Reverse (Drive) / Forward (Pedro) |
| **Odometry** | Pinpoint Computer | `"pinpoint"` | `GoBildaPinpointDriver` | 4-bar pods (X: +1.89", Y: -3.88") |
| **Intake** | Intake Roller Motor | `"intake"` | `DcMotorEx` | Reverse direction, power cached |
| **Intake** | Intake Deploy Servo | `"servo_intake"` | `Servo` | Pos A (0.21, Up), Pos B (0.90, Down) |
| **Shooter** | Flywheel Motor | `"flywheel"` | `DcMotorEx` | RUN_USING_ENCODER, target HIVE cell |
| **Shooter** | Feeder Motor | `"feeder"` | `DcMotorEx` | Indexes elements to flywheel |
| **Hood** | Elevation Angle Servo | `"hood"` | `Servo` | 0.0 to 0.5 (0.1 step increments) |
| **Gate / Blocker** | Element Gate Servo | `"servo_blocker"` | `Servo` | 0.4 (Blocked), 0.0 (Open) |
| **Sweeper** | Flower Sweeper | `"sweeper"` | `CRServo` | Continuous rotation: 0.8 Down, -0.8 Up |
| **Status LED** | Indicator Light | `"led"` | `Servo` (Blinkin PWM) | Visual feedback (Violet, Blue, Yellow, White) |

---

## 3. Software Architecture & Execution Loop

### 3.1. `RobotOpMode` Loop Order
All OpModes extend [`RobotOpMode`](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/RobotOpMode.java):
1. **Bulk Cache Clear**: `LynxModule.clearBulkCache()` clears manual cache once per loop (batches all I2C/Lynx bus reads).
2. **`onStart()`**: Executes once on the first cycle after START.
3. **`CommandScheduler.getInstance().run()`**:
   - Subsystem `periodic()` calls run.
   - Button/trigger listeners evaluate.
   - Active commands execute.
4. **Telemetry & LEDs**: Telemetry is logged and updated; loop frequency (`loopHz`) is tracked.

### 3.2. Coordinate Systems & Field Geometry
- **Field Size**: 144" × 144" square playing field with Pedro coordinate conventions.
- **Blue Alliance Coordinates**:
  - Default Start: `(X = 132.0, Y = 9.0, 90°)` (or `(103.0, 134.0, 90°)` for Back/Front variants)
  - Goal Position: `(X = 84.0, Y = 71.0)`
  - Shooting Pose: `(X = 84.0, Y = 121.0, 90°)`
  - Element Pickup: `(X = 132.8, Y = 134.3, 0°)`
  - Flower Dispenser: `(X = 113.0, Y = 92.0, 0°)` $\rightarrow$ `(X = 130.0, Y = 93.0, 0°)`
  - Park Pose: `(X = 131.0, Y = 46.0, 90°)`
- **Red Alliance Coordinates**:
  - Default Start: `(X = 9.0, Y = 132.0, 90°)` (or `(39.0, 8.8, 270°)` for Front variants)
  - Goal Position: `(X = 58.0, Y = 71.0)`
  - Shooting Pose: `(X = 60.0, Y = 20.0, 270°)`
  - Element Pickup: `(X = 7.6, Y = 7.6, 180°)`
  - Flower Dispenser: `(X = 30.0, Y = 46.0, 270°)` $\rightarrow$ `(X = 11.0, Y = 46.0, 180°)`
  - Park Pose: `(X = 10.0, Y = 95.0, 270°)`
- **Auto-Aim Calculation**: Calculates angle error from current odometry pose `(x, y, heading)` to alliance goal `(goalX, goalY)` and turns using closed-loop PD + $\mathrm{kS}$ feedforward while allowing driver translation.

### 3.3. Control Mapping (Gamepad 1)
- **Left Stick (X/Y)**: Field-centric translation (with acceleration/deceleration smoothing).
- **Right Stick X**: Manual rotation (switches automatically to Heading Hold when released).
- **Right Trigger (Hold)**: **Auto-Aim & Shoot Prep** (rotates robot to goal, opens blocker, lowers intake to B, powers feeder & intake).
- **Left Trigger (Hold)**: Direct intake & queue sequence (intake to B, intake motor on, feeder on).
- **Right Bumper**: Toggle sweeper down / return for flower dispensers.
- **Left Bumper**: Toggle feeder motor ON/OFF.
- **Button Y**: Toggle flywheel ON/OFF.
- **Button A**: Toggle intake lift servo position (Position A $\leftrightarrow$ Position B).
- **Button B**: Toggle intake motor ON/OFF.
- **D-Pad Up / Down**: Blocker servo to Blocked (`0.4`) / Open (`0.0`).
- **D-Pad Right / Left**: Increment / decrement shooter hood angle by 0.1.
- **Options**: Re-zero robot odometry pose to the starting position.

### 3.4. LED Status Visual Feedback (Blinkin PWM)
- **VIOLET (`0.333`)**: Auto-Aim active and tracking goal.
- **BLUE (`0.388`)**: Intake deployed (Position B) and active.
- **YELLOW (`0.673`)**: Flywheel spinning / ready to shoot.
- **GREEN (`0.611`)**: Initialized & ready for start.
- **WHITE (`0.277`)**: Default idle / teleop driving.
- **RED (`0.653`)**: Warning / blocker engaged / red alliance idle.

---

## 4. Coding & Agent Guidelines

1. **Discuss Before Implementing**: Present proposed changes, explain trade-offs, and get user confirmation before writing or modifying code.
2. **Inherit from `RobotOpMode`**: Never use raw `LinearOpMode` or unmanaged loops.
3. **Cache Hardware Writes**: Avoid sending redundant `setPower()` or `setPosition()` calls if values haven't changed.
4. **FTCLib 2.1.1 Nesting Caveat**: Never nest a `CommandGroup` (such as `SequentialCommandGroup`) inside a `ParallelCommandGroup` in FTCLib 2.1.1 (known scheduler freeze/crash). Chain multiple trigger listeners instead.
5. **Keep `BaseTeleOp` Unified**: All shared mechanism bindings belong in [`BaseTeleOp`](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/teleops/BaseTeleOp.java). Alliance-specific classes (`TeleOp1` for Blue, `TeleOp2` for Red) only override start poses, goals, and alliance colors.

