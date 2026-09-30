package frc.BotchoCheese.Commands;

import java.util.Locale;
import java.util.Optional;
import java.util.OptionalDouble;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.BotchoCheese.Constants.ShotConstants;
import frc.BotchoCheese.Subsystems.Feeder;
import frc.BotchoCheese.Subsystems.Indexer;
import frc.BotchoCheese.Subsystems.Intake;
import frc.BotchoCheese.Subsystems.RangeSensor;
import frc.BotchoCheese.Subsystems.Shooter;
import frc.BotchoCheese.Utils.ShotPhysics;
import frc.BotchoCheese.Utils.ShotPhysics.ShotSolution;

/**
 * Shot whose flywheel speed is solved from the range sensor's distance to the hub.
 *
 * <p>{@link #update()} runs every robot loop (shooting or not) so the dashboard always shows the shot
 * the robot would take from where it is standing. {@link #command()} spins the shooter to that speed,
 * waits until it is actually there, then feeds. With no valid range reading it falls back to the
 * regular fixed-speed shot so the operator always gets a ball out.
 */
public final class RangedShot {
    private static final String STATUS_KEY = "RangedShot/Status";
    private static final String HUB_DISTANCE_KEY = "RangedShot/HubDistanceMeters";
    private static final String EXIT_VELOCITY_KEY = "RangedShot/ExitVelocityMps";
    private static final String FLIGHT_TIME_KEY = "RangedShot/FlightTimeSeconds";
    private static final String ENTRY_ANGLE_KEY = "RangedShot/EntryAngleDeg";
    private static final String KINETIC_ENERGY_KEY = "RangedShot/KineticEnergyJ";
    private static final String MOMENTUM_KEY = "RangedShot/MomentumKgMps";
    private static final String LAUNCH_FORCE_KEY = "RangedShot/AvgLaunchForceN";
    private static final String CONTACT_TIME_KEY = "RangedShot/ContactTimeMs";
    private static final String SURFACE_SPEED_KEY = "RangedShot/WheelSurfaceSpeedMps";
    private static final String TARGET_RPS_KEY = "RangedShot/TargetRps";
    private static final String USING_FALLBACK_KEY = "RangedShot/UsingFallback";
    // Live-tunable: ball exit speed / wheel surface speed. Raise it if shots go long, lower it if short.
    private static final String EFFICIENCY_KEY = "Shots/Ranged Efficiency";

    private final Shooter shooter;
    private final RangeSensor rangeSensor;
    private final Intake intake;
    private final Indexer indexer;
    private final Feeder feeder;
    private final double intakeDuty;
    private final double indexerDuty;
    private final double feederDuty;

    private double targetRps = ShotConstants.FALLBACK_SHOOTER_RPS;

    public RangedShot(
        Shooter shooter,
        RangeSensor rangeSensor,
        Intake intake,
        Indexer indexer,
        Feeder feeder,
        double intakeDuty,
        double indexerDuty,
        double feederDuty
    ) {
        this.shooter = shooter;
        this.rangeSensor = rangeSensor;
        this.intake = intake;
        this.indexer = indexer;
        this.feeder = feeder;
        this.intakeDuty = intakeDuty;
        this.indexerDuty = indexerDuty;
        this.feederDuty = feederDuty;

        SmartDashboard.putNumber(EFFICIENCY_KEY, ShotConstants.DEFAULT_SURFACE_SPEED_RATIO);
        update();
    }

    /** Flywheel RPS the shot is currently asking for (both wheels get the same speed). */
    public double getTargetRps() {
        return targetRps;
    }

    /** Recompute the shot from the latest range reading and publish every intermediate quantity. */
    public void update() {
        OptionalDouble sensorDistance = rangeSensor.getDistanceMeters();
        if (sensorDistance.isEmpty()) {
            useFallback("NO RANGE READING - FALLBACK " + formatRps(ShotConstants.FALLBACK_SHOOTER_RPS));
            return;
        }

        // Sensor face -> hub surface, corrected to launch point -> aim point.
        double hubDistanceMeters = sensorDistance.getAsDouble()
            - ShotConstants.RANGE_SENSOR_AHEAD_OF_LAUNCH_METERS
            + ShotConstants.HUB_SURFACE_TO_TARGET_METERS;
        double surfaceSpeedRatio = MathUtil.clamp(
            SmartDashboard.getNumber(EFFICIENCY_KEY, ShotConstants.DEFAULT_SURFACE_SPEED_RATIO), 0.05, 1.0);

        Optional<ShotSolution> solution = ShotPhysics.solve(
            ShotConstants.launcherModel(surfaceSpeedRatio), hubDistanceMeters);
        if (solution.isEmpty()) {
            useFallback(String.format(
                Locale.US,
                "UNREACHABLE AT %.0f DEG FROM %.2f m - FALLBACK %s",
                ShotConstants.LAUNCH_ANGLE_DEG,
                hubDistanceMeters,
                formatRps(ShotConstants.FALLBACK_SHOOTER_RPS)));
            return;
        }

        ShotSolution shot = solution.get();
        publishSolution(shot);
        if (shot.flywheelSetpointRps() > ShotConstants.MAX_SHOOTER_RPS) {
            targetRps = ShotConstants.MAX_SHOOTER_RPS;
            SmartDashboard.putString(
                STATUS_KEY,
                "TOO FAR: NEEDS " + formatRps(shot.flywheelSetpointRps())
                    + ", CAPPED AT " + formatRps(ShotConstants.MAX_SHOOTER_RPS));
        } else {
            targetRps = shot.flywheelSetpointRps();
            SmartDashboard.putString(STATUS_KEY, "OK");
        }
        SmartDashboard.putNumber(TARGET_RPS_KEY, targetRps);
        SmartDashboard.putBoolean(USING_FALLBACK_KEY, false);
    }

    /** Spin to the solved speed, feed once the wheels are there (or after the spin-up timeout). */
    public Command command() {
        return Commands.parallel(
            shooter.shootRps(this::getTargetRps, this::getTargetRps),
            Commands.sequence(
                Commands.waitUntil(() -> shooter.isAtTargetSpeed(ShotConstants.AT_SPEED_TOLERANCE_RPS))
                    .withTimeout(ShotConstants.RANGED_SPINUP_TIMEOUT_SECONDS),
                Commands.parallel(
                    intake.runIntake(intakeDuty),
                    indexer.runIndexer(indexerDuty),
                    feeder.runFeeder(feederDuty)
                )
            )
        ).withName("RangedShot");
    }

    private void useFallback(String status) {
        targetRps = ShotConstants.FALLBACK_SHOOTER_RPS;
        SmartDashboard.putString(STATUS_KEY, status);
        SmartDashboard.putNumber(TARGET_RPS_KEY, targetRps);
        SmartDashboard.putBoolean(USING_FALLBACK_KEY, true);
    }

    private static void publishSolution(ShotSolution shot) {
        SmartDashboard.putNumber(HUB_DISTANCE_KEY, shot.horizontalDistanceMeters());
        SmartDashboard.putNumber(EXIT_VELOCITY_KEY, shot.exitVelocityMps());
        SmartDashboard.putNumber(FLIGHT_TIME_KEY, shot.flightTimeSeconds());
        SmartDashboard.putNumber(ENTRY_ANGLE_KEY, shot.entryAngleDeg());
        SmartDashboard.putNumber(KINETIC_ENERGY_KEY, shot.kineticEnergyJoules());
        SmartDashboard.putNumber(MOMENTUM_KEY, shot.momentumKgMps());
        SmartDashboard.putNumber(LAUNCH_FORCE_KEY, shot.averageLaunchForceNewtons());
        SmartDashboard.putNumber(CONTACT_TIME_KEY, shot.contactTimeSeconds() * 1000.0);
        SmartDashboard.putNumber(SURFACE_SPEED_KEY, shot.wheelSurfaceSpeedMps());
    }

    private static String formatRps(double rps) {
        return String.format(Locale.US, "%.0f RPS", rps);
    }
}
