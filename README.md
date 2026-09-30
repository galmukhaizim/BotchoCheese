# Team 1515 Mortorq - 2026 Rebuilt Robot Code

This README is the current quick-reference for drivers, operators, pit crew, and programmers.

## Robot Summary

- Drivetrain is CTRE Phoenix 6 swerve with field-centric default driving.
- Autonomous modes are loaded from `src/main/deploy/pathplanner/autos/`.
- Teleop no longer resets pose from the selected auto when enabled.
- Pivot is manual only. There is no working auto-home routine in the current code.
- Limelight pose fusion is present in code but currently disabled.
- A CANrange time-of-flight sensor aimed at the hub feeds the physics-solved ranged shot (operator left bumper). See "Ranged Shot Physics" below.

## Current Controller Bindings

### Driver Controller (`port 0`)

- Left stick `Y`: drive forward and backward.
- Left stick `X`: strafe left and right.
- Right stick `X`: rotate robot.
- X (hold): swerve brake.
- D-pad up/down/left/right (hold): robot-centric crawl at fixed speed.
- Left trigger (hold): slow rotate left (in place).
- Right trigger (hold): slow rotate right (in place).
- Start (press): reseed field-centric heading only.

### Operator Controller (`port 1`)

- D-pad up (hold): pivot up.
- D-pad down (hold): pivot down.
- Left trigger (hold): intake / anti-jam sequence.
- B (hold): reverse intake, indexer, and feeder.
- Right trigger (toggle): big shot.
- Right bumper (toggle): regular shot.
- Left bumper (toggle): ranged shot. Flywheel speed is solved from the range sensor's distance to the hub; feeds once the wheels are at speed.
- X (toggle): SmartDashboard-programmed shot using `Shots/X Back RPS` and `Shots/X Front RPS`.
- Y (toggle): lob shot.
- A (toggle): line-drive shot.

## Operator Notes

- Left trigger is hold-to-run. Releasing it stops the intake sequence.
- `B` is hold-to-run. Releasing it stops the reverse/un-jam sequence.
- Right trigger, right bumper, left bumper, `X`, `Y`, and `A` are toggled shots. Press once to start, press again to stop.
- The ranged shot (left bumper) falls back to the regular 90 RPS shot when the range sensor has no valid reading. `RangedShot/Status` on the dashboard says which one you are getting.
- Pivot control is manual. Do not expect it to home itself or move to saved positions.

## Autonomous

- Dashboard chooser key: `Auto Chooser`.
- On autonomous enable, the robot seeds pose from the selected auto's starting pose when one is available.
- If no auto is selected, the robot runs a no-op command in autonomous.
- Deploy now deletes old files from the roboRIO deploy directory, which helps prevent stale autos and paths from lingering between events.

### Auto readiness indicators

- `Auto/SelectedValid`: `true` when a real auto is selected, `false` when the chooser is still on `Select Auto`.
- `Auto/SelectedName`: the currently selected auto name.
- `Auto/Status`: human-readable autonomous status and failure reason.
- `Auto/StartPoseSeeded`: `true` once the selected auto's start pose has been seeded successfully.

## Dashboard Items

- `Auto Chooser`: autonomous chooser.
- `Auto/SelectedValid`: quick validity check for the selected auto.
- `Auto/SelectedName`: selected auto name.
- `Auto/Status`: autonomous readiness / failure status.
- `Auto/StartPoseSeeded`: whether autonomous start pose seeding succeeded.
- `Match/TimeSeconds`: current match time from DS/FMS.
- `Match/Mode`: one of `DISABLED`, `AUTO`, `TELEOP`, `TEST`, `E-STOP`.
- `Match/Alliance`: alliance color (`Red`, `Blue`, or `Unknown`).
- `Match/Station`: station number (`1`-`3`, or `Unknown`).
- `Match/Type`, `Match/Number`, `Match/Replay`, `Match/EventName`: match metadata from DS/FMS.
- `Match/GameData`: raw game data character from FMS (`R` or `B` for REBUILT).
- `Match/RebuiltShift`: active REBUILT phase (`SHIFT_1`..`SHIFT_4`, `TRANSITION`, `ENDGAME`, plus non-teleop mode labels).
- `Match/RebuiltNextShift`: next REBUILT phase (`SHIFT_1`..`SHIFT_4`, `ENDGAME`, `MATCH_END`).
- `Match/RebuiltActiveForUs`: `true` when your alliance is active in the current REBUILT shift logic.
- `Match/RebuiltShiftTimeLeftSeconds`: countdown in seconds until the next REBUILT phase boundary.
- `Match/CoachSummary`: compact drive-coach summary string (`<shift> -> <next> | ACTIVE/INACTIVE | <seconds> left`).
- `Shots/X Back RPS`: custom back shooter speed for the `X` shot.
- `Shots/X Front RPS`: custom front shooter speed for the `X` shot.
- `Shots/Ranged Efficiency`: live-tunable ball-speed-to-wheel-surface-speed ratio for the ranged shot (see "Ranged Shot Physics").
- `RangeSensor/DistanceMeters`, `RangeSensor/Valid`, `RangeSensor/Health`, `RangeSensor/SignalStrength`, `RangeSensor/UsingHeldReading`: raw CANrange state.
- `RangedShot/Status`: `OK`, `TOO FAR: ...` (capped at max RPS), `UNREACHABLE ...`, or `NO RANGE READING - FALLBACK ...`.
- `RangedShot/HubDistanceMeters`: launch-point-to-aim-point horizontal distance used by the solver.
- `RangedShot/ExitVelocityMps`, `RangedShot/FlightTimeSeconds`, `RangedShot/EntryAngleDeg`: the solved trajectory.
- `RangedShot/KineticEnergyJ`, `RangedShot/MomentumKgMps`, `RangedShot/AvgLaunchForceN`, `RangedShot/ContactTimeMs`: what the wheels must give the ball.
- `RangedShot/WheelSurfaceSpeedMps`, `RangedShot/TargetRps`: the flywheel command that results.
- `RangedShot/UsingFallback`: `true` when the fixed 90 RPS fallback is in use instead of the solved speed.
- `Swerve/* Raw Abs (rot)`: raw absolute encoder values for each swerve module.
- `SwerveCal/* OffsetToPaste (rot)`: copy these directly into `TunerConstants` encoder offsets (Option 1 calibration flow).
- `SwerveCal/PasteLine *`: per-module ready-to-paste lines for `TunerConstants`.
- `SwerveCal/PasteBlock`: four-line ready-to-paste block for all module offsets.
- `SwerveCal/Instruction`: quick reminder of the calibration workflow.

## Elastic Layout From Robot

- The robot hosts deploy files on port `5800` (`WebServer.start(5800, Filesystem.getDeployDirectory().getPath())`).
- `elastic-layout.json` is deployed at `src/main/deploy/elastic-layout.json`, so drive team can load directly from robot.
- In Elastic, use `File -> Load Layout From Robot`.
- Direct URL fallback: `http://roborio-<team>-frc.local:5800/elastic-layout.json` (replace `<team>` with your team number).

## Shooter Tuning Knobs

- Shared shoot conveyor duty cycles and shooter spin-up timeout now live in `RobotContainer`:
- `SHOOT_INTAKE_DUTY`
- `SHOOT_INDEXER_DUTY`
- `SHOOT_FEEDER_DUTY`
- `SHOOTER_SPINUP_TIMEOUT_SECONDS`
- These values are used by both operator shoot buttons and the registered PathPlanner `Shoot` named command.

## Ranged Shot Physics

The ranged shot (operator left bumper, PathPlanner named command `ShootRanged`) replaces the fixed shooter RPS with a value solved from Newtonian mechanics every robot loop:

1. **Distance.** The CANrange reads sensor-face-to-hub-surface distance. `ShotConstants` corrects that to launch-point-to-aim-point horizontal distance `d`.
2. **Trajectory.** For the fixed launch angle `θ` and height rise `Δh`, the exit velocity that passes through the aim point is `v² = g d² / (2 cos²θ (d tanθ − Δh))`. Flight time and entry angle come from the same equations.
3. **Energy, momentum, force.** `KE = ½ m v²`, `p = m v`, and over the wheel contact arc `s` the work-energy theorem gives the average launch force `F = KE / s` with contact time `t = 2 s / v`.
4. **Flywheel setpoint.** The wheel surface must still move at `v / efficiency` when the ball leaves, and by then the wheels have lost `KE / energyTransferEfficiency` of rotational energy, so the speed to hold beforehand is `ω₀ = sqrt(ω_exit² + 2 KE / (η I))`. Heavier balls therefore need faster wheels, not just more force.

All of this lives in `src/main/java/frc/BotchoCheese/Utils/ShotPhysics.java` (pure math, unit-tested in `src/test/java/.../ShotPhysicsTest.java`) and `src/main/java/frc/BotchoCheese/Constants/ShotConstants.java` (the numbers).

**Every value in `ShotConstants` marked PLACEHOLDER is a guess.** Before trusting the shot, measure and replace:

- `BALL_MASS_KG` (weigh a ball; 0.25 kg assumed).
- `LAUNCH_ANGLE_DEG`, `LAUNCH_HEIGHT_METERS`, `TARGET_HEIGHT_METERS`.
- `SHOOTER_WHEEL_RADIUS_METERS`, `FLYWHEEL_INERTIA_KG_M2`, `CONTACT_LENGTH_METERS`.
- `RANGE_SENSOR_AHEAD_OF_LAUNCH_METERS`, `HUB_SURFACE_TO_TARGET_METERS` (where the sensor sits and what it actually sees).

Then calibrate the one live knob, `Shots/Ranged Efficiency` (ball exit speed / wheel surface speed). Its default (0.226) was fitted so the model asks for the proven 90 RPS at the 2.5 m shooting spot with the placeholders above, which gives roughly 82 RPS at 1.5 m up to 106 RPS at 4 m. Whenever a placeholder changes, park at 2.5 m, read `RangedShot/TargetRps`, and adjust the ratio until it reads 90 again; then test shots at other distances. Shots going long mean the ratio is too low; short means too high. Copy the final value into `DEFAULT_SURFACE_SPEED_RATIO`.

Limits: the model ignores air drag (fine at these speeds and distances), the CANrange sees at most 4 m, and the solver caps its request at `MAX_SHOOTER_RPS` (120) and reports `TOO FAR` on the dashboard when it wanted more.

## CAN Motor Map And Config

| CAN ID | Device | Subsystem / Module | Bus | Motor Type / Arrangement | Stator Limit (A) | Supply Limit (A) | Neutral Mode | Key Config Notes |
|---|---|---|---|---|---:|---:|---|---|
| 0 | Front Left Drive | Swerve Front Left | CANivore (`1515Canivore`) | `TalonFX_Integrated` drive | Not explicitly set in code | Not explicitly set in code | Coast | Left side drive invert flag `false` |
| 1 | Front Left Steer | Swerve Front Left | CANivore (`1515Canivore`) | `TalonFX_Integrated` steer | 45 | Not explicitly set in code | Coast | Steer invert `false`, feedback source `FusedCANcoder` |
| 2 | Front Right Drive | Swerve Front Right | CANivore (`1515Canivore`) | `TalonFX_Integrated` drive | Not explicitly set in code | Not explicitly set in code | Coast | Right side drive invert flag `true` |
| 3 | Front Right Steer | Swerve Front Right | CANivore (`1515Canivore`) | `TalonFX_Integrated` steer | 45 | Not explicitly set in code | Coast | Steer invert `false`, feedback source `FusedCANcoder` |
| 4 | Back Left Drive | Swerve Back Left | CANivore (`1515Canivore`) | `TalonFX_Integrated` drive | Not explicitly set in code | Not explicitly set in code | Coast | Left side drive invert flag `false` |
| 5 | Back Left Steer | Swerve Back Left | CANivore (`1515Canivore`) | `TalonFX_Integrated` steer | 45 | Not explicitly set in code | Coast | Steer invert `false`, feedback source `FusedCANcoder` |
| 6 | Back Right Drive | Swerve Back Right | CANivore (`1515Canivore`) | `TalonFX_Integrated` drive | Not explicitly set in code | Not explicitly set in code | Coast | Right side drive invert flag `true` |
| 7 | Back Right Steer | Swerve Back Right | CANivore (`1515Canivore`) | `TalonFX_Integrated` steer | 45 | Not explicitly set in code | Coast | Steer invert `false`, feedback source `FusedCANcoder` |
| 18 | Indexer Motor | Indexer | roboRIO CAN | `TalonFXS` + `Minion_JST` | 40 | 30 | Brake | Duty-cycle output command |
| 20 | Left Pivot Motor (Leader) | Pivot | roboRIO CAN | `TalonFX` | 60 | 40 | Brake by config; switched to Coast in `disabledInit()` | Manual voltage control; follower pair |
| 21 | Right Pivot Motor (Follower) | Pivot | roboRIO CAN | `TalonFX` follower (Opposed) | 60 | 40 | Brake by config; switched to Coast in `disabledInit()` | Follows ID 20 with `MotorAlignmentValue.Opposed` |
| 22 | Intake Motor | Intake | roboRIO CAN | `TalonFX` (X44) | 60 | 40 | Brake | Inverted `Clockwise_Positive`; duty-cycle output |
| 23 | Feeder Motor | Feeder | roboRIO CAN | `TalonFX` | 60 | 40 | Brake | Duty-cycle output |
| 24 | Back Left Shooter | Shooter | roboRIO CAN | `TalonFX` | 40 | 30 | Brake | Inverted `Clockwise_Positive`; velocity closed-loop |
| 25 | Back Right Shooter (Follower) | Shooter | roboRIO CAN | `TalonFX` follower (Aligned) | 40 | 30 | Brake | Follows ID 24 with `MotorAlignmentValue.Aligned` |
| 26 | Front Shooter | Shooter | roboRIO CAN | `TalonFX` | 40 | 30 | Brake | Inverted `Clockwise_Positive`; velocity closed-loop |

### Swerve Azimuth Sensors (Not Motors)

| CAN ID | Device | Module | Bus | Notes |
|---|---|---|---|---|
| 10 | CANcoder | Front Left | CANivore (`1515Canivore`) | Encoder invert flag `false`; code-side offset in `TunerConstants` |
| 11 | CANcoder | Front Right | CANivore (`1515Canivore`) | Encoder invert flag `false`; code-side offset in `TunerConstants` |
| 12 | CANcoder | Back Left | CANivore (`1515Canivore`) | Encoder invert flag `false`; code-side offset in `TunerConstants` |
| 13 | CANcoder | Back Right | CANivore (`1515Canivore`) | Encoder invert flag `false`; code-side offset in `TunerConstants` |

### Other Sensors

| CAN ID | Device | Purpose | Bus | Notes |
|---|---|---|---|---|
| 30 | Pigeon 2 | Drivetrain heading | CANivore (`1515Canivore`) | Configured through `TunerConstants` |
| 41 | CANrange | Distance to hub for the ranged shot | roboRIO CAN | Long-range mode, 50 Hz, 6.75 deg FOV, detection threshold 4 m. Mount it aimed at the hub wall along the shot direction; readings outside 0.3-4 m are ignored |

## Vision Notes

- `LimelightHomography.update(...)` is currently disabled in `Robot`.
- The codebase still contains some vision alignment commands, but they are not currently bound to the driver controller.

## Important Files

- `src/main/java/frc/BotchoCheese/RobotContainer.java`: controller bindings and command wiring.
- `src/main/java/frc/BotchoCheese/Robot.java`: robot mode lifecycle behavior.
- `src/main/java/frc/BotchoCheese/Subsystems/Pivot.java`: manual pivot behavior.
- `src/main/java/frc/BotchoCheese/Subsystems/RangeSensor.java`: CANrange wrapper with validity checks and reading hold.
- `src/main/java/frc/BotchoCheese/Commands/RangedShot.java`: range-sensor shot command and its dashboard output.
- `src/main/java/frc/BotchoCheese/Utils/ShotPhysics.java` and `Constants/ShotConstants.java`: the shot model and its physical constants.
- `src/main/deploy/pathplanner/`: autonomous and path assets.

## Team Workflow Notes

- Keep this README in sync with `RobotContainer.java`.
- If bindings change, update this document in the same PR.
- Treat `controllerbounds.txt` as potentially stale unless it is updated alongside the code.
