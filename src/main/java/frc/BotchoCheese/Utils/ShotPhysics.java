package frc.BotchoCheese.Utils;

import java.util.Optional;

/**
 * Newtonian model of one shot from the flywheel shooter.
 *
 * <p>Given the horizontal distance to the target, this works out the ball's required exit velocity
 * from projectile motion (no air drag), then uses energy and momentum to get the launch force and the
 * flywheel speed the motors must hold so the ball actually leaves at that velocity. The ball's mass
 * enters twice: it sets the kinetic energy/force needed, and it sets how much the flywheels slow down
 * while they hand that energy to the ball.
 *
 * <p>All math is pure and static so it can be unit tested without robot hardware. Units are SI unless
 * a name says otherwise.
 */
public final class ShotPhysics {
    public static final double GRAVITY_MPS2 = 9.80665;

    private ShotPhysics() {}

    /** Physical description of the launcher and the game piece. Build one from {@code ShotConstants}. */
    public record LauncherModel(
        /** Launch angle above horizontal. */
        double launchAngleRad,
        /** Height of the ball's center as it leaves the shooter. */
        double launchHeightMeters,
        /** Height of the aim point (the opening the ball must pass through). */
        double targetHeightMeters,
        double ballMassKg,
        /** Radius of the wheels that touch the ball. */
        double wheelRadiusMeters,
        /** Combined rotational inertia of every wheel and rotor that gives up energy to the ball. */
        double flywheelInertiaKgM2,
        /** Arc length over which the wheels push on the ball before it leaves. */
        double contactLengthMeters,
        /** Ball exit speed divided by wheel surface speed at the instant of exit (slip/compression loss). */
        double surfaceSpeedRatio,
        /** Fraction of the flywheels' lost rotational energy that ends up as ball kinetic energy. */
        double energyTransferEfficiency
    ) {}

    /** Everything the drive team and the shooter need to know about one shot. */
    public record ShotSolution(
        double horizontalDistanceMeters,
        double exitVelocityMps,
        double flightTimeSeconds,
        /** Angle of the ball's velocity as it reaches the target; negative means descending. */
        double entryAngleDeg,
        double kineticEnergyJoules,
        double momentumKgMps,
        /** Average force the wheels apply to the ball over the contact length (work-energy theorem). */
        double averageLaunchForceNewtons,
        double contactTimeSeconds,
        /** Wheel surface speed needed at the instant the ball leaves. */
        double wheelSurfaceSpeedMps,
        /** Wheel speed to hold before the ball arrives, so it is still fast enough after slowing down. */
        double flywheelSetpointRps
    ) {}

    /**
     * Solve for the shot that passes through the aim point at the given horizontal distance.
     *
     * @return empty when the target is unreachable at this launch angle or the inputs make no sense.
     */
    public static Optional<ShotSolution> solve(LauncherModel model, double horizontalDistanceMeters) {
        double d = horizontalDistanceMeters;
        double theta = model.launchAngleRad();
        double cos = Math.cos(theta);
        double sin = Math.sin(theta);
        // A launch angle must point forward and upward: straight up (or beyond) can never cover any distance.
        if (!(d > 0.0) || !(theta > 0.0 && theta < Math.PI / 2.0) || !(model.ballMassKg() >= 0.0)
            || !(model.wheelRadiusMeters() > 0.0) || !(model.flywheelInertiaKgM2() > 0.0)
            || !(model.contactLengthMeters() > 0.0) || !(model.surfaceSpeedRatio() > 0.0)
            || !(model.energyTransferEfficiency() > 0.0)) {
            return Optional.empty();
        }

        // Projectile motion: x = v cos(theta) t, y = v sin(theta) t - g t^2 / 2.
        // Eliminating t at x = d and requiring y = rise gives v^2 = g d^2 / (2 cos^2(theta) (d tan(theta) - rise)).
        double rise = model.targetHeightMeters() - model.launchHeightMeters();
        double drop = d * Math.tan(theta) - rise;
        if (drop <= 0.0) {
            // The straight-line path never gets above the target, so no speed can reach it at this angle.
            return Optional.empty();
        }
        double v2 = GRAVITY_MPS2 * d * d / (2.0 * cos * cos * drop);
        double v = Math.sqrt(v2);
        double flightTime = d / (v * cos);
        double verticalVelocityAtTarget = v * sin - GRAVITY_MPS2 * flightTime;
        double entryAngleDeg = Math.toDegrees(Math.atan2(verticalVelocityAtTarget, v * cos));

        // Energy and momentum the ball must be given.
        double m = model.ballMassKg();
        double kineticEnergy = 0.5 * m * v2;
        double momentum = m * v;
        // Work-energy theorem over the contact arc: F * s = 1/2 m v^2.
        double averageForce = kineticEnergy / model.contactLengthMeters();
        // Constant acceleration from rest over the contact arc: s = 1/2 v t.
        double contactTime = 2.0 * model.contactLengthMeters() / v;

        // Flywheel: the wheel surface must still be moving at v / ratio when the ball leaves, and by then the
        // wheels have given up (kinetic energy / efficiency) of rotational energy: 1/2 I (w0^2 - w1^2).
        double surfaceSpeed = v / model.surfaceSpeedRatio();
        double omegaExit = surfaceSpeed / model.wheelRadiusMeters();
        double omegaSetpoint = Math.sqrt(
            omegaExit * omegaExit
                + 2.0 * kineticEnergy / (model.energyTransferEfficiency() * model.flywheelInertiaKgM2()));
        double setpointRps = omegaSetpoint / (2.0 * Math.PI);

        return Optional.of(new ShotSolution(
            d,
            v,
            flightTime,
            entryAngleDeg,
            kineticEnergy,
            momentum,
            averageForce,
            contactTime,
            surfaceSpeed,
            setpointRps
        ));
    }
}
