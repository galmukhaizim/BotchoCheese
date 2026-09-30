package frc.BotchoCheese.Constants;

import frc.BotchoCheese.Utils.ShotPhysics.LauncherModel;

/**
 * Physical constants for the range-sensor shot (see {@link frc.BotchoCheese.Utils.ShotPhysics}).
 *
 * <p>Every value marked PLACEHOLDER is a realistic guess, not a measurement. Measure it on the robot
 * and replace it; the shot will only be as accurate as these numbers.
 */
public final class ShotConstants {
    private ShotConstants() {}

    // ---- Game piece ----
    // PLACEHOLDER: weigh a ball. 0.25 kg is typical for a foam FRC ball.
    public static final double BALL_MASS_KG = 0.25;

    // ---- Launch geometry ----
    // PLACEHOLDER: angle of the ball's exit path above horizontal. Fixed by the shooter's exit geometry.
    public static final double LAUNCH_ANGLE_DEG = 60.0;
    // PLACEHOLDER: height of the ball's center as it leaves the shooter, robot on the floor.
    public static final double LAUNCH_HEIGHT_METERS = 0.50;
    // PLACEHOLDER: height of the hub opening the ball must pass through.
    public static final double TARGET_HEIGHT_METERS = 1.80;

    // ---- Flywheel ----
    // PLACEHOLDER: radius of the shooter wheels that touch the ball (4 in wheel assumed).
    public static final double SHOOTER_WHEEL_RADIUS_METERS = 0.0508;
    // PLACEHOLDER: combined inertia of all shooter wheels and rotors. Three 4 in, 0.3 kg wheels give
    // about 3 * (1/2 * 0.3 * 0.0508^2) = 0.0012 kg m^2; rotors and hubs add a little more.
    public static final double FLYWHEEL_INERTIA_KG_M2 = 0.0015;
    // PLACEHOLDER: arc length the wheels push the ball through before it leaves.
    public static final double CONTACT_LENGTH_METERS = 0.10;
    // PLACEHOLDER: ball exit speed / wheel surface speed. 0.226 is the value that makes this model ask for the
    // proven 90 RPS at the 2.5 m shooting spot with the other placeholders above, so the ranged shot is a drop-in
    // there and only differs at other distances. Tune live via `Shots/Ranged Efficiency`: shots going long mean
    // the ratio is too low, short means too high. Re-fit it whenever another placeholder changes.
    public static final double DEFAULT_SURFACE_SPEED_RATIO = 0.226;
    // PLACEHOLDER: fraction of flywheel energy loss that becomes ball kinetic energy (rest is compression heat).
    public static final double ENERGY_TRANSFER_EFFICIENCY = 0.5;
    // Highest RPS the shooter is asked for; matches the existing big shot.
    public static final double MAX_SHOOTER_RPS = 120.0;
    // Used when there is no valid range reading; matches the existing regular shot.
    public static final double FALLBACK_SHOOTER_RPS = 90.0;

    // ---- Range sensor geometry ----
    // PLACEHOLDER: how far the sensor face sits ahead of the ball's launch point, along the shot direction.
    // Positive when the sensor is closer to the hub than the launch point.
    public static final double RANGE_SENSOR_AHEAD_OF_LAUNCH_METERS = 0.0;
    // PLACEHOLDER: horizontal distance from the hub surface the sensor sees to the aim point behind it.
    public static final double HUB_SURFACE_TO_TARGET_METERS = 0.0;
    // Readings outside this window are ignored (sensor noise, a passing robot, or nothing in view).
    public static final double MIN_VALID_RANGE_METERS = 0.3;
    public static final double MAX_VALID_RANGE_METERS = 4.0;
    // How long the last good reading is trusted after the sensor loses the hub.
    public static final double RANGE_HOLD_SECONDS = 1.0;

    // ---- Shot flow ----
    public static final double AT_SPEED_TOLERANCE_RPS = 2.0;
    public static final double RANGED_SPINUP_TIMEOUT_SECONDS = 1.5;

    /** Build the launcher model, taking the live-tunable surface speed ratio as a parameter. */
    public static LauncherModel launcherModel(double surfaceSpeedRatio) {
        return new LauncherModel(
            Math.toRadians(LAUNCH_ANGLE_DEG),
            LAUNCH_HEIGHT_METERS,
            TARGET_HEIGHT_METERS,
            BALL_MASS_KG,
            SHOOTER_WHEEL_RADIUS_METERS,
            FLYWHEEL_INERTIA_KG_M2,
            CONTACT_LENGTH_METERS,
            surfaceSpeedRatio,
            ENERGY_TRANSFER_EFFICIENCY
        );
    }
}
