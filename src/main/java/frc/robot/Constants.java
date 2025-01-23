package frc.robot;

import static com.ctre.phoenix6.signals.NeutralModeValue.Brake;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.lib.control.PIDConfig;
import frc.robot.generated.TunerConstants;

public class Constants {

    public static final class SwerveConstants {
        public static final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // m/s
        public static final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
    }

    public static final class CoralConstants {


        public static final SparkMaxConfig CORAL_INTAKECONFIG = new SparkMaxConfig();
        public static final TalonFXConfiguration CORAL_WRISTCONFIG = new TalonFXConfiguration();

        public static final double CORAL_INTAKE_GEAR_RATIO = 1 / 5;
        public static final double CORAL_WRIST_LENGTH = 34.176;
        public static final double CORAL_WRIST_GEAR_RATIO = 3 / 275;

        // TODO : Need to be Tuned.
        public static final SimpleMotorFeedforward CORAL_INTAKE_FEED_FORWARD = new SimpleMotorFeedforward(0, 0, 0);
        public static final ArmFeedforward CORAL_WRIST_FEED_FORWARD = new ArmFeedforward(0, 0, 0,0);

        public static final PIDConfig CORAL_INTAKE_FEEDBACK = new PIDConfig(0, 0, 0);
        public static final PIDConfig CORAL_WRIST_FEEDBACK = new PIDConfig(0, 0, 0);


        static {
            CORAL_INTAKECONFIG.encoder.positionConversionFactor(CORAL_INTAKE_GEAR_RATIO);
            CORAL_INTAKECONFIG.encoder.velocityConversionFactor(CORAL_INTAKE_GEAR_RATIO);

            CORAL_INTAKECONFIG.idleMode(IdleMode.kBrake).smartCurrentLimit(40).voltageCompensation(12);
            CORAL_INTAKECONFIG.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // Set PID values for position control
                    // TODO: Need To be Tuned.
                    .apply(CORAL_INTAKE_FEEDBACK.createSparkMaxConfig())
                    .iZone(0)
                    .outputRange(-1, 1)
                    .maxMotion
                    // Set MAXMotion parameters for position control
                    .maxVelocity(2000)
                    .maxAcceleration(10000)
                    .allowedClosedLoopError(0.25);
        }

        static {
            CORAL_WRIST_FEEDBACK.updatePidConfig(CORAL_WRISTCONFIG);
            CORAL_WRISTCONFIG.Voltage
                    .withPeakForwardVoltage(12)
                    .withPeakReverseVoltage(-12);
            CORAL_WRISTCONFIG.CurrentLimits.StatorCurrentLimitEnable = true;
            CORAL_WRISTCONFIG.CurrentLimits.StatorCurrentLimit = 80;
            CORAL_WRISTCONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            CORAL_WRISTCONFIG.MotorOutput.NeutralMode = Brake;
            CORAL_WRISTCONFIG.Feedback.SensorToMechanismRatio = CORAL_WRIST_GEAR_RATIO;
            CORAL_WRISTCONFIG.ClosedLoopGeneral.ContinuousWrap = true;

            CORAL_WRISTCONFIG.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
            CORAL_WRISTCONFIG.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.4;
            CORAL_WRISTCONFIG.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
            CORAL_WRISTCONFIG.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.4;

        }
    }

    public static final class AlgaeConstants {


        public static final SparkMaxConfig ALGAE_INTAKECONFIG = new SparkMaxConfig();
        public static final TalonFXConfiguration ALGAE_WRISTCONFIG = new TalonFXConfiguration();

        public static final double ALGAE_INTAKE_GEAR_RATIO = 1 / 5;
        public static final double WRIST_LENGTH = 34.176;
        public static final double ALGAE_WRIST_GEAR_RATIO = 3 / 275;

        // TODO : Need to be Tuned.
        public static final SimpleMotorFeedforward INTAKE_FEED_FORWARD = new SimpleMotorFeedforward(0, 0, 0);
        public static final ArmFeedforward WRIST_FEED_FORWARD = new ArmFeedforward(0, 0, 0,0);

        public static final PIDConfig ALGAE_INTAKE_FEEDBACK = new PIDConfig(0, 0, 0);
        public static final PIDConfig ALGAE_WRIST_FEEDBACK = new PIDConfig(0, 0, 0);


        static {
            ALGAE_INTAKECONFIG.encoder.positionConversionFactor(ALGAE_INTAKE_GEAR_RATIO);
            ALGAE_INTAKECONFIG.encoder.velocityConversionFactor(ALGAE_INTAKE_GEAR_RATIO);

            ALGAE_INTAKECONFIG.idleMode(IdleMode.kBrake).smartCurrentLimit(40).voltageCompensation(12);
            ALGAE_INTAKECONFIG.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // Set PID values for position control
                    // TODO: Need To be Tuned.
                    .apply(ALGAE_INTAKE_FEEDBACK.createSparkMaxConfig())
                    .iZone(0)
                    .outputRange(-1, 1)
                    .maxMotion
                    // Set MAXMotion parameters for position control
                    .maxVelocity(2000)
                    .maxAcceleration(10000)
                    .allowedClosedLoopError(0.25);
        }

        static {
            ALGAE_WRIST_FEEDBACK.updatePidConfig(ALGAE_WRISTCONFIG);
            ALGAE_WRISTCONFIG.Voltage
                    .withPeakForwardVoltage(12)
                    .withPeakReverseVoltage(-12);
            ALGAE_WRISTCONFIG.CurrentLimits.StatorCurrentLimitEnable = true;
            ALGAE_WRISTCONFIG.CurrentLimits.StatorCurrentLimit = 80;
            ALGAE_WRISTCONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            ALGAE_WRISTCONFIG.MotorOutput.NeutralMode = Brake;
            ALGAE_WRISTCONFIG.Feedback.SensorToMechanismRatio = ALGAE_WRIST_GEAR_RATIO;
            ALGAE_WRISTCONFIG.ClosedLoopGeneral.ContinuousWrap = true;

            ALGAE_WRISTCONFIG.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
            ALGAE_WRISTCONFIG.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.4;
            ALGAE_WRISTCONFIG.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
            ALGAE_WRISTCONFIG.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.4;

        }
    }






}
    
    

