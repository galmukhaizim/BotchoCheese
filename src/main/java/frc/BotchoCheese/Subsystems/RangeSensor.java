package frc.BotchoCheese.Subsystems;

import java.util.OptionalDouble;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.MeasurementHealthValue;
import com.ctre.phoenix6.signals.UpdateModeValue;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.BotchoCheese.Constants.RobotMap;
import frc.BotchoCheese.Constants.ShotConstants;

/**
 * CTRE CANrange time-of-flight sensor aimed at the hub.
 *
 * <p>Reads the straight-line distance to the nearest surface in the sensor's field of view. The shot
 * code only trusts a reading when the sensor reports a detection with good health inside the valid
 * window, and it keeps the last good reading for a short hold time so a ball or robot crossing the beam
 * does not yank the shooter speed around mid-shot.
 */
public class RangeSensor extends SubsystemBase {
    private static final String RANGE_METERS_KEY = "RangeSensor/DistanceMeters";
    private static final String RANGE_VALID_KEY = "RangeSensor/Valid";
    private static final String RANGE_HEALTH_KEY = "RangeSensor/Health";
    private static final String RANGE_SIGNAL_STRENGTH_KEY = "RangeSensor/SignalStrength";
    private static final String RANGE_HELD_KEY = "RangeSensor/UsingHeldReading";
    // Long-range mode is needed to see the hub from the 2.5 m shooting spots; 50 Hz is its maximum rate.
    private static final double UPDATE_FREQUENCY_HZ = 50.0;
    // Narrow the beam so the sensor reports the hub, not a neighbouring robot or the floor.
    private static final double FOV_RANGE_DEG = 6.75;
    private static final double PROXIMITY_HYSTERESIS_METERS = 0.05;

    private final CANrange canrange = new CANrange(RobotMap.RANGE_SENSOR_CAN_ID);
    private final StatusSignal<Distance> distanceSignal;
    private final StatusSignal<Boolean> detectedSignal;
    private final StatusSignal<Double> signalStrengthSignal;
    private final StatusSignal<MeasurementHealthValue> healthSignal;

    private double lastValidDistanceMeters = Double.NaN;
    private double lastValidTimestampSeconds = Double.NEGATIVE_INFINITY;
    private boolean currentReadingValid = false;

    public RangeSensor() {
        CANrangeConfiguration config = new CANrangeConfiguration();
        config.ToFParams.UpdateMode = UpdateModeValue.LongRangeUserFreq;
        config.ToFParams.UpdateFrequency = UPDATE_FREQUENCY_HZ;
        // Anything inside the valid window counts as "detected"; the window's far edge is the sensor's limit.
        config.ProximityParams.ProximityThreshold = ShotConstants.MAX_VALID_RANGE_METERS;
        config.ProximityParams.ProximityHysteresis = PROXIMITY_HYSTERESIS_METERS;
        config.FovParams.FOVRangeX = FOV_RANGE_DEG;
        config.FovParams.FOVRangeY = FOV_RANGE_DEG;
        canrange.getConfigurator().apply(config);

        distanceSignal = canrange.getDistance();
        detectedSignal = canrange.getIsDetected();
        signalStrengthSignal = canrange.getSignalStrength();
        healthSignal = canrange.getMeasurementHealth();
        BaseStatusSignal.setUpdateFrequencyForAll(
            UPDATE_FREQUENCY_HZ, distanceSignal, detectedSignal, signalStrengthSignal, healthSignal);
    }

    /** Raw sensor-face-to-surface distance, valid or not. NaN when the signal has never arrived. */
    public double getRawDistanceMeters() {
        return distanceSignal.getValueAsDouble();
    }

    /** True when the most recent sample passed every validity check. */
    public boolean isCurrentReadingValid() {
        return currentReadingValid;
    }

    /**
     * Best available distance from the sensor face to the hub surface: the live reading when valid,
     * otherwise the last valid reading if it is younger than {@link ShotConstants#RANGE_HOLD_SECONDS}.
     */
    public OptionalDouble getDistanceMeters() {
        if (Double.isNaN(lastValidDistanceMeters)) {
            return OptionalDouble.empty();
        }
        if (Timer.getFPGATimestamp() - lastValidTimestampSeconds > ShotConstants.RANGE_HOLD_SECONDS) {
            return OptionalDouble.empty();
        }
        return OptionalDouble.of(lastValidDistanceMeters);
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(distanceSignal, detectedSignal, signalStrengthSignal, healthSignal);

        double distanceMeters = distanceSignal.getValueAsDouble();
        MeasurementHealthValue health = healthSignal.getValue();
        // "Limited" still means a usable reading of a far target; only "Bad" is rejected outright.
        currentReadingValid = distanceSignal.getStatus().isOK()
            && detectedSignal.getValue()
            && health != MeasurementHealthValue.Bad
            && distanceMeters >= ShotConstants.MIN_VALID_RANGE_METERS
            && distanceMeters <= ShotConstants.MAX_VALID_RANGE_METERS;
        if (currentReadingValid) {
            lastValidDistanceMeters = distanceMeters;
            lastValidTimestampSeconds = Timer.getFPGATimestamp();
        }

        SmartDashboard.putNumber(RANGE_METERS_KEY, distanceMeters);
        SmartDashboard.putBoolean(RANGE_VALID_KEY, currentReadingValid);
        SmartDashboard.putString(RANGE_HEALTH_KEY, health == null ? "NO SIGNAL" : health.name());
        SmartDashboard.putNumber(RANGE_SIGNAL_STRENGTH_KEY, signalStrengthSignal.getValueAsDouble());
        SmartDashboard.putBoolean(RANGE_HELD_KEY, !currentReadingValid && getDistanceMeters().isPresent());
    }
}
