package frc.BotchoCheese.Utils;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Optional;

import org.junit.jupiter.api.Test;

import frc.BotchoCheese.Utils.ShotPhysics.LauncherModel;
import frc.BotchoCheese.Utils.ShotPhysics.ShotSolution;

class ShotPhysicsTest {
    private static final double TOLERANCE = 1e-3;
    private static final double BALL_MASS = 0.25;
    private static final double WHEEL_RADIUS = 0.05;
    private static final double INERTIA = 0.002;
    private static final double CONTACT_LENGTH = 0.1;

    private static LauncherModel model(double angleDeg, double launchHeight, double targetHeight, double mass) {
        return new LauncherModel(
            Math.toRadians(angleDeg), launchHeight, targetHeight, mass,
            WHEEL_RADIUS, INERTIA, CONTACT_LENGTH, 1.0, 1.0);
    }

    @Test
    void levelShotAtFortyFiveDegreesMatchesTextbookRange() {
        // Range on level ground at 45 deg is v^2 / g, so v = sqrt(g d).
        double d = 5.0;
        ShotSolution shot = ShotPhysics.solve(model(45.0, 1.0, 1.0, BALL_MASS), d).orElseThrow();

        double expectedV = Math.sqrt(ShotPhysics.GRAVITY_MPS2 * d);
        assertEquals(expectedV, shot.exitVelocityMps(), TOLERANCE);
        assertEquals(d / (expectedV * Math.cos(Math.toRadians(45.0))), shot.flightTimeSeconds(), TOLERANCE);
        // Symmetric arc: it comes in at the same angle it went out, descending.
        assertEquals(-45.0, shot.entryAngleDeg(), TOLERANCE);
    }

    @Test
    void energyMomentumAndForceFollowFromExitVelocity() {
        ShotSolution shot = ShotPhysics.solve(model(45.0, 1.0, 1.0, BALL_MASS), 5.0).orElseThrow();
        double v = shot.exitVelocityMps();

        assertEquals(0.5 * BALL_MASS * v * v, shot.kineticEnergyJoules(), TOLERANCE);
        assertEquals(BALL_MASS * v, shot.momentumKgMps(), TOLERANCE);
        // F * s = 1/2 m v^2, and s = 1/2 v t from rest under constant acceleration.
        assertEquals(shot.kineticEnergyJoules() / CONTACT_LENGTH, shot.averageLaunchForceNewtons(), TOLERANCE);
        assertEquals(2.0 * CONTACT_LENGTH / v, shot.contactTimeSeconds(), TOLERANCE);
        // Impulse check: average force times contact time equals the momentum delivered.
        assertEquals(shot.momentumKgMps(), shot.averageLaunchForceNewtons() * shot.contactTimeSeconds(), TOLERANCE);
    }

    @Test
    void masslessBallNeedsOnlyTheSurfaceSpeed() {
        ShotSolution shot = ShotPhysics.solve(model(45.0, 1.0, 1.0, 0.0), 5.0).orElseThrow();
        double expectedRps = shot.wheelSurfaceSpeedMps() / WHEEL_RADIUS / (2.0 * Math.PI);
        assertEquals(expectedRps, shot.flywheelSetpointRps(), TOLERANCE);
    }

    @Test
    void heavierBallNeedsFasterFlywheel() {
        ShotSolution light = ShotPhysics.solve(model(45.0, 1.0, 1.0, 0.1), 5.0).orElseThrow();
        ShotSolution heavy = ShotPhysics.solve(model(45.0, 1.0, 1.0, 0.5), 5.0).orElseThrow();

        // Same trajectory, so the same exit velocity...
        assertEquals(light.exitVelocityMps(), heavy.exitVelocityMps(), TOLERANCE);
        // ...but the heavier ball drains more flywheel energy, so the wheels must start faster.
        assertTrue(heavy.flywheelSetpointRps() > light.flywheelSetpointRps());
        assertTrue(heavy.averageLaunchForceNewtons() > light.averageLaunchForceNewtons());
    }

    @Test
    void flywheelSetpointConservesEnergy() {
        ShotSolution shot = ShotPhysics.solve(model(60.0, 0.5, 1.8, BALL_MASS), 2.5).orElseThrow();
        double omega0 = shot.flywheelSetpointRps() * 2.0 * Math.PI;
        double omegaExit = shot.wheelSurfaceSpeedMps() / WHEEL_RADIUS;
        // 1/2 I (w0^2 - w1^2) is the energy handed to the ball (efficiency is 1.0 in this model).
        double flywheelEnergyLost = 0.5 * INERTIA * (omega0 * omega0 - omegaExit * omegaExit);
        assertEquals(shot.kineticEnergyJoules(), flywheelEnergyLost, TOLERANCE);
    }

    @Test
    void lowerSurfaceSpeedRatioNeedsFasterWheels() {
        LauncherModel efficient = new LauncherModel(
            Math.toRadians(60.0), 0.5, 1.8, BALL_MASS, WHEEL_RADIUS, INERTIA, CONTACT_LENGTH, 1.0, 1.0);
        LauncherModel slippy = new LauncherModel(
            Math.toRadians(60.0), 0.5, 1.8, BALL_MASS, WHEEL_RADIUS, INERTIA, CONTACT_LENGTH, 0.5, 1.0);

        double efficientRps = ShotPhysics.solve(efficient, 2.5).orElseThrow().flywheelSetpointRps();
        double slippyRps = ShotPhysics.solve(slippy, 2.5).orElseThrow().flywheelSetpointRps();
        assertTrue(slippyRps > efficientRps);
    }

    @Test
    void targetAboveTheLaunchLineIsUnreachable() {
        // At 30 deg over 1 m the launch line only rises 0.577 m, so a 1 m rise cannot be hit at any speed.
        Optional<ShotSolution> shot = ShotPhysics.solve(model(30.0, 0.0, 1.0, BALL_MASS), 1.0);
        assertTrue(shot.isEmpty());
    }

    @Test
    void nonsenseInputsAreRejected() {
        assertTrue(ShotPhysics.solve(model(45.0, 0.0, 0.0, BALL_MASS), 0.0).isEmpty());
        assertTrue(ShotPhysics.solve(model(45.0, 0.0, 0.0, BALL_MASS), -1.0).isEmpty());
        assertTrue(ShotPhysics.solve(model(90.0, 0.0, 0.0, BALL_MASS), 2.0).isEmpty());
        assertTrue(ShotPhysics.solve(model(45.0, 0.0, 0.0, BALL_MASS), Double.NaN).isEmpty());
    }
}
