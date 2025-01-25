package frc.robot;

import static com.ctre.phoenix6.signals.NeutralModeValue.Brake;
import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.lib.control.PIDConfig;
import frc.robot.generated.TunerConstants;

public class Constants {

    public enum Mode {
        SIM,
        COMP,
        REPLAY
    }

    public static final class SwerveConstants {
        public static final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // m/s
        public static final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
    }

    public static final class CoralConstants {


        public static final SparkMaxConfig CORAL_INTAKECONFIG = new SparkMaxConfig();
        public static final TalonFXConfiguration CORAL_WRISTCONFIG = new TalonFXConfiguration();

        public static final double CORAL_INTAKE_GEAR_RATIO = 1 ;
        public static final double CORAL_WRIST_LENGTH = 34.176;
        public static final double CORAL_WRIST_GEAR_RATIO = 3 / 275;

        // TODO : Need to be Tuned.
        public static final SimpleMotorFeedforward CORAL_INTAKE_FEED_FORWARD = new SimpleMotorFeedforward(0, 0, 0);
        public static final ArmFeedforward CORAL_WRIST_FEED_FORWARD = new ArmFeedforward(0, 0, 0,0);

        public static final PIDConfig CORAL_INTAKE_FEEDBACK = new PIDConfig(0, 0, 0);
        public static final PIDConfig CORAL_WRIST_FEEDBACK = new PIDConfig(0, 0, 0);


        static {
            CORAL_INTAKECONFIG.encoder.positionConversionFactor(CORAL_INTAKE_GEAR_RATIO);
            CORAL_INTAKECONFIG.encoder.velocityConversionFactor(CORAL_INTAKE_GEAR_RATIO/60);

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


    public static final class ElevatorConstants {


        public static final SparkMaxConfig ALGAE_INTAKECONFIG = new SparkMaxConfig();
        public static final TalonFXConfiguration ELEVATOR_CONFIG_L = new TalonFXConfiguration();
        public static final TalonFXConfiguration ELEVATOR_CONFIG_R = new TalonFXConfiguration();

        public static final double ELEVATOR_LENGTH = 60/Math.PI;
        public static final double ELEVATOR_GEAR_RATIO = 1/9;

        // TODO : Need to be Tuned.
        public static final ElevatorFeedforward ELEVATOR_FEED_FORWARD = new ElevatorFeedforward(0, 0, 0,0);


        public static final PIDConfig ELEVATOR_WRIST_FEEDBACK = new PIDConfig(0, 0, 0);

        //TODO : Need to be Tuned. & add degrees(?)
        public enum ElevatorState {
            L1(5.0),
            L2(80.0),
            L3(120.0),
            L4(160.0),
            NORMAL(0.0),
            STATION(100.0);

            public double height;
            private ElevatorState(double height) {
                this.height = height;
            }

        }


        static {
            ELEVATOR_WRIST_FEEDBACK.updatePidConfig(ELEVATOR_CONFIG_L);
            ELEVATOR_CONFIG_L.Voltage
                    .withPeakForwardVoltage(12)
                    .withPeakReverseVoltage(-12);
            ELEVATOR_CONFIG_L.CurrentLimits
                    .withStatorCurrentLimitEnable(true)
                    .withStatorCurrentLimit(100);
            ELEVATOR_CONFIG_L.MotorOutput
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(Brake);
            ELEVATOR_CONFIG_L.Feedback.SensorToMechanismRatio = ELEVATOR_GEAR_RATIO;
            ELEVATOR_CONFIG_L.ClosedLoopGeneral.ContinuousWrap = true;

            ELEVATOR_CONFIG_L.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
            ELEVATOR_CONFIG_L.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 100;
            ELEVATOR_CONFIG_L.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
            ELEVATOR_CONFIG_L.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.4;
        }

        static {
            ELEVATOR_WRIST_FEEDBACK.updatePidConfig(ELEVATOR_CONFIG_R);
            ELEVATOR_CONFIG_R.Voltage
                    .withPeakForwardVoltage(12)
                    .withPeakReverseVoltage(-12);
            ELEVATOR_CONFIG_R.CurrentLimits
                    .withStatorCurrentLimitEnable(true)
                    .withStatorCurrentLimit(100);
            ELEVATOR_CONFIG_R.MotorOutput
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(Brake);
            ELEVATOR_CONFIG_R.Feedback.SensorToMechanismRatio = ELEVATOR_GEAR_RATIO;
            ELEVATOR_CONFIG_R.ClosedLoopGeneral.ContinuousWrap = true;

            ELEVATOR_CONFIG_R.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
            ELEVATOR_CONFIG_R.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 15.3;
            ELEVATOR_CONFIG_R.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
            ELEVATOR_CONFIG_R.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.3;
        }
    }

    public static final class HangConstants {
        public static final SparkFlexConfig CATCH_HANG_CONFIG = new SparkFlexConfig();
        public static final TalonFXConfiguration HANG_CONFIG = new TalonFXConfiguration();

        public static final double HANG_LENGTH = 34.176;
        public static final double HANG_GEAR_RATIO = 3 / 275;

        // TODO : Need to be Tuned.
        public static final SimpleMotorFeedforward HANG_FEED_FORWARD = new SimpleMotorFeedforward(0, 0, 0);
        public static final ArmFeedforward CATCH_HANG_FEED_FORWARD = new ArmFeedforward(0, 0, 0,0);

        public static final PIDConfig HANG_FEEDBACK = new PIDConfig(0, 0, 0);
        public static final PIDConfig CATCH_HANG_FEEDBACK = new PIDConfig(0, 0, 0);

        static {
            CATCH_HANG_CONFIG.encoder.positionConversionFactor(HANG_GEAR_RATIO);
            CATCH_HANG_CONFIG.encoder.velocityConversionFactor(HANG_GEAR_RATIO / 60);

            CATCH_HANG_CONFIG.idleMode(IdleMode.kBrake).smartCurrentLimit(40).voltageCompensation(12);
            CATCH_HANG_CONFIG.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // Set PID values for position control
                    // TODO: Need To be Tuned.
                    .apply(HANG_FEEDBACK.createSparkMaxConfig())
                    .iZone(0)
                    .outputRange(-1, 1)
                    .maxMotion
                    // Set MAXMotion parameters for position control
                    .maxVelocity(2000)
                    .maxAcceleration(10000)
                    .allowedClosedLoopError(0.25);
        }
        static {
            CATCH_HANG_FEEDBACK.updatePidConfig(HANG_CONFIG);
            HANG_CONFIG.Voltage
                    .withPeakForwardVoltage(12)
                    .withPeakReverseVoltage(-12);
            HANG_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;
            HANG_CONFIG.CurrentLimits.StatorCurrentLimit = 80;
            HANG_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            HANG_CONFIG.MotorOutput.NeutralMode = Brake;
            HANG_CONFIG.Feedback.SensorToMechanismRatio = HANG_GEAR_RATIO;
            HANG_CONFIG.ClosedLoopGeneral.ContinuousWrap = true;

            HANG_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
            HANG_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.4;
            HANG_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
            HANG_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.4;
        }

        public static final class GrabSubsystem {

            public static final SparkFlexConfig GRAB_WRIST_CONFIG = new SparkFlexConfig();
            public static final SparkFlexConfig GRAB_INTAKE_CONFIG = new SparkFlexConfig();

            public static final double GRAB_WRIST_GEAR_RATIO = 1 / 5;
            public static final double GRAB_INTAKE_GEAR_RATIO = 1 / 5;
            public static final double CORAL_WRIST_LENGTH = 34.176;
            public static final double CORAL_WRIST_GEAR_RATIO = 3 / 275;

            // TODO : Need to be Tuned.
            public static final SimpleMotorFeedforward GRAB_INTAKE_FEED_FORWARD = new SimpleMotorFeedforward(0, 0, 0);
            public static final ArmFeedforward GRAB_WRIST_FEED_FORWARD = new ArmFeedforward(0, 0, 0,0);

            public static final PIDConfig GRAB_INTAKE_FEEDBACK = new PIDConfig(0, 0, 0);
            public static final PIDConfig GRAB_WRIST_FEEDBACK = new PIDConfig(0, 0, 0);


            static {
                GRAB_INTAKE_CONFIG.encoder.positionConversionFactor(GRAB_INTAKE_GEAR_RATIO);
                GRAB_INTAKE_CONFIG.encoder.velocityConversionFactor(GRAB_INTAKE_GEAR_RATIO /60);

                GRAB_INTAKE_CONFIG.idleMode(IdleMode.kBrake).smartCurrentLimit(40).voltageCompensation(12);
                GRAB_INTAKE_CONFIG.closedLoop
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        // Set PID values for position control
                        .apply(GRAB_INTAKE_FEEDBACK.createSparkMaxConfig())
                        .iZone(0)
                        .outputRange(-1, 1)
                        .maxMotion
                        // Set MAXMotion parameters for position control
                        .maxVelocity(2000)
                        .maxAcceleration(10000)
                        .allowedClosedLoopError(0.25);
            }
            static {
                GRAB_WRIST_CONFIG.encoder.positionConversionFactor(GRAB_WRIST_GEAR_RATIO);
                GRAB_WRIST_CONFIG.encoder.velocityConversionFactor(GRAB_WRIST_GEAR_RATIO /60);

                GRAB_WRIST_CONFIG.idleMode(IdleMode.kBrake).smartCurrentLimit(60).voltageCompensation(12);
                GRAB_WRIST_CONFIG.closedLoop
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        // Set PID values for position control
                        // TODO: Need To be Tuned.
                        .apply(GRAB_WRIST_FEEDBACK.createSparkMaxConfig())
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







}
    
    

