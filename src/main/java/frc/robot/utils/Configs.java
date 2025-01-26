package frc.robot.utils;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.utils.Constants.ModuleConstants;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public final class Configs {
    public static final class MK4iSwerveModule {
        public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
        public static final SparkMaxConfig turningConfig = new SparkMaxConfig();
        static {
            // Use module constants to calculate conversion factors and feed forward gain.
            double drivingFactor = ModuleConstants.wheelDiameterMeters * Math.PI
                    / ModuleConstants.drivingMotorReduction;
            double turningFactor = 2 * Math.PI;
            double drivingVelocityFeedForward = 1 / ModuleConstants.driveWheelFreeSpeedRps;
            drivingConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(50);
            drivingConfig.encoder
                    .positionConversionFactor(drivingFactor) // meters
                    .velocityConversionFactor(drivingFactor / 60.0); // meters per second
            drivingConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(0.04, 0, 0)
                    .velocityFF(drivingVelocityFeedForward)
                    .outputRange(-1, 1);
            turningConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(20);
            turningConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
                    // These are example gains you may need to tune them for your own robot!
                    .pid(1, 0, 0)
                    .outputRange(-1, 1)
                    // Enable PID wrap around for the turning motor. This will allow the PID
                    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
                    // to 10 degrees will go through 0 rather than the other direction which is a
                    // longer route.
                    .positionWrappingEnabled(true)
                    .positionWrappingInputRange(0, turningFactor);
        }
    }
}