package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.lib.util.CANDeviceId;
import frc.robot.generated.TunerConstants;

public class Constants {

    public static final class SwerveConstants {
        public static final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // m/s
        public static final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
    }

    public static final class CoralConstants {
        public static final CANDeviceId INTAKE_ID = new CANDeviceId(1);
        public static final CANDeviceId ARM_ID = new CANDeviceId(2);
        public static final CANDeviceId WRIST_ID = new CANDeviceId(3);

        public static final SparkMaxConfig intakeConfig = new SparkMaxConfig();
        public static final SparkFlexConfig armConfig = new SparkFlexConfig();
        public static final SparkMaxConfig wristConfig = new SparkMaxConfig();

        static {
            intakeConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40).voltageCompensation(12);

            intakeConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                // Set PID values for position control
                // TODO: Need To be Tuned.
                .pidf(0.1, 0, 0, 0)
                .iZone(0) 
                .outputRange(-1, 1)
                .maxMotion
                // Set MAXMotion parameters for position control
                .maxVelocity(2000)
                .maxAcceleration(10000)
                .allowedClosedLoopError(0.25);
        }

    }
    
    
    
}
