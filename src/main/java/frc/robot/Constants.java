package frc.robot;

import static com.ctre.phoenix6.signals.NeutralModeValue.Brake;
import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
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

        public static final double CORAL_INTAKE_GEAR_RATIO = 3.0;
        public static final double CORAL_WRIST_LENGTH = 34.176;
        public static final double CORAL_WRIST_GEAR_RATIO = 225.0/4.0;

        // TODO : Need to be Tuned.
        public static final PIDConfig CORAL_INTAKE_FEEDBACK = new PIDConfig(0.0005, 0, 0.00001,0.0292);
        public static final PIDConfig CORAL_WRIST_FEEDBACK = new PIDConfig(24.244, 0, 0.13351);

        public static final double CORAL_WRIST_KS = 0.2793/12;
        public static final double CORAL_WRIST_KV = 6.2584/12;
        public static final double CORAL_WRIST_KA = 0.061275/12;
        public static final double CORAL_WRIST_KG = 0.016319/12;

        public static final double CORAL_INTAKE_VELOCITY = 30.0;
        
        public static final double CORAL_WRIST_FORWARD_SOFT_LIMIT = 90.0;
        public static final double CORAL_WRIST_REVERSE_SOFT_LIMIT = -60.0;

        public static final double CORAL_ENCODER_OFFSET = 0.124755859375;


        public static final SparkMaxConfig CORAL_INTAKECONFIG = new SparkMaxConfig();
        public static final TalonFXConfiguration CORAL_WRISTCONFIG = new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimitEnable(true)
                    .withStatorCurrentLimit(100))
            .withVoltage(new VoltageConfigs()
                    .withPeakForwardVoltage(12)
                    .withPeakReverseVoltage(-12))
            .withFeedback(new FeedbackConfigs()
                    .withSensorToMechanismRatio(CORAL_WRIST_GEAR_RATIO))
                    .withClosedLoopGeneral(new ClosedLoopGeneralConfigs()
                    .withContinuousWrap(true))
            .withMotorOutput(new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(Brake))

            .withSlot0(new Slot0Configs()
                    .withKP(CORAL_WRIST_FEEDBACK.P)
                    .withKI(CORAL_WRIST_FEEDBACK.I)
                    .withKD(CORAL_WRIST_FEEDBACK.D)
                    .withGravityType(GravityTypeValue.Arm_Cosine)
                    .withKS(CORAL_WRIST_KS)
                    .withKV(CORAL_WRIST_KV)
                    .withKA(CORAL_WRIST_KA)
                    .withKG(CORAL_WRIST_KG))
            .withMotionMagic(new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(7.5)
                    .withMotionMagicAcceleration(15)
                    .withMotionMagicJerk(150))
            .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
                    .withForwardSoftLimitEnable(true)
                    .withForwardSoftLimitThreshold(Units.degreesToRotations(CORAL_WRIST_FORWARD_SOFT_LIMIT))
                    .withReverseSoftLimitEnable(true)
                    .withReverseSoftLimitThreshold(Units.degreesToRotations(CORAL_WRIST_REVERSE_SOFT_LIMIT)));

        public static final CANcoderConfiguration CORAL_WRIST_ENCODER_CONFIG = new CANcoderConfiguration()
            .withMagnetSensor(new MagnetSensorConfigs()
                .withMagnetOffset(CORAL_ENCODER_OFFSET)
                .withAbsoluteSensorDiscontinuityPoint(0.5)
                .withSensorDirection(SensorDirectionValue.Clockwise_Positive));

        


        static {
            CORAL_INTAKECONFIG.encoder.positionConversionFactor(1.0/CORAL_INTAKE_GEAR_RATIO);
            CORAL_INTAKECONFIG.encoder.velocityConversionFactor(1.0/CORAL_INTAKE_GEAR_RATIO/60.0);

            CORAL_INTAKECONFIG.idleMode(IdleMode.kCoast);
            CORAL_INTAKECONFIG.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // Set PID values for elevatorPosition control
                    // TODO: Need To be Tuned.
                    .apply(CORAL_INTAKE_FEEDBACK.createSparkMaxConfig())
                    .iZone(0)
                    .outputRange(-1, 1);
        }
    }


    public static final class ElevatorConstants {

       

        public static final double ELEVATOR_LENGTH = 0.06/Math.PI;
        public static final double ELEVATOR_GEAR_RATIO = 9.0;

        // TODO : Need to be Tuned.
        public static final ElevatorFeedforward ELEVATOR_FEED_FORWARD = new ElevatorFeedforward(0.4, 0, 0.4,0);

        public static final double ELEVATOR_MAX_LENGTH = 1.3;
        public static final double ELEVATOR_MIN_LENGTH = 0.01;

        public static final double ELEVATOR_KS = 0.26859/12;
        public static final double ELEVATOR_KV = 10.923/12;
        public static final double ELEVATOR_KA = 1.7629/12;
        public static final double ELEVATOR_KG = 0.24228/12;


        public static final PIDConfig ELEVATOR_FEEDBACK = new PIDConfig(58.629, 0, 16.814);

        public static final TalonFXConfiguration ELEVATOR_CONFIG_L = new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimitEnable(true)
                    .withStatorCurrentLimit(100))
                .withVoltage(new VoltageConfigs()
                    .withPeakForwardVoltage(12)
                    .withPeakReverseVoltage(-12))
                .withFeedback(new FeedbackConfigs()
                    .withSensorToMechanismRatio(ELEVATOR_GEAR_RATIO))
                    .withClosedLoopGeneral(new ClosedLoopGeneralConfigs()
                    .withContinuousWrap(true))
                .withMotorOutput(new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(Brake))

                .withSlot0(new Slot0Configs()
                    .withKP(ELEVATOR_FEEDBACK.P)
                    .withKI(ELEVATOR_FEEDBACK.I)
                    .withKD(ELEVATOR_FEEDBACK.D)
                    .withGravityType(GravityTypeValue.Elevator_Static)
                    .withKG(ELEVATOR_KG)
                    .withKS(ELEVATOR_KS)
                    .withKV(ELEVATOR_KV)
                    .withKA(ELEVATOR_KA))

                .withMotionMagic(new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(5.0)
                    .withMotionMagicAcceleration(10.0)
                    .withMotionMagicJerk(100))
                .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
                    .withForwardSoftLimitEnable(true)
                    .withForwardSoftLimitThreshold(ELEVATOR_MAX_LENGTH/ELEVATOR_LENGTH*ELEVATOR_GEAR_RATIO/(2*Math.PI))
                    .withReverseSoftLimitEnable(true)
                    .withReverseSoftLimitThreshold(ELEVATOR_MIN_LENGTH/ELEVATOR_LENGTH*ELEVATOR_GEAR_RATIO/(2*Math.PI))
                );
            
            public static final TalonFXConfiguration ELEVATOR_CONFIG_R = new TalonFXConfiguration()
                .withCurrentLimits(
                    new CurrentLimitsConfigs()
                        .withStatorCurrentLimitEnable(true)
                        .withStatorCurrentLimit(100))
                    .withVoltage(new VoltageConfigs()
                        .withPeakForwardVoltage(12)
                        .withPeakReverseVoltage(-12))
                    .withFeedback(new FeedbackConfigs()
                        .withSensorToMechanismRatio(ELEVATOR_GEAR_RATIO))
                        .withClosedLoopGeneral(new ClosedLoopGeneralConfigs()
                        .withContinuousWrap(true))
                    .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(InvertedValue.Clockwise_Positive)
                        .withNeutralMode(Brake))
    
                    .withSlot0(new Slot0Configs()
                        .withKP(ELEVATOR_FEEDBACK.P)
                        .withKI(ELEVATOR_FEEDBACK.I)
                        .withKD(ELEVATOR_FEEDBACK.D)
                        .withGravityType(GravityTypeValue.Elevator_Static)
                        .withKG(ELEVATOR_KG)
                        .withKS(ELEVATOR_KS)
                        .withKV(ELEVATOR_KV)
                        .withKA(ELEVATOR_KA))
                    .withMotionMagic(new MotionMagicConfigs()
                        .withMotionMagicCruiseVelocity(5.0)
                        .withMotionMagicAcceleration(10.0)
                        .withMotionMagicJerk(100))
                    .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
                        .withForwardSoftLimitEnable(true)
                        .withForwardSoftLimitThreshold(ELEVATOR_MAX_LENGTH/ELEVATOR_LENGTH*ELEVATOR_GEAR_RATIO/(2*Math.PI))
                        .withReverseSoftLimitEnable(true)
                        .withReverseSoftLimitThreshold(ELEVATOR_MIN_LENGTH/ELEVATOR_LENGTH*ELEVATOR_GEAR_RATIO/(2*Math.PI))
                    );


        //TODO : Need to be Tuned. & add degrees(?)
        public static enum ScoreState {
            L1(0.05, 60.0),
            L2(0.4, 30.0),
            L3(0.8, 0.0),
            L4(1.2, -45.0),
            NORMAL(0.0, 0.0),
            STATION(1.0, 0.0),;

            public double elevatorPosition;
            public double armPosition;
            private ScoreState(double elevatorPosition,double armPosition) {
                this.elevatorPosition = elevatorPosition;
                this.armPosition = armPosition;
            }
        }
    }

    public static final class HangConstants {
        public static final SparkFlexConfig CATCH_HANG_CONFIG = new SparkFlexConfig();
        public static final TalonFXConfiguration HANG_CONFIG = new TalonFXConfiguration();

        public static final double HANG_LENGTH = 34.176;
        public static final double HANG_GEAR_RATIO = 275.0/3.0;
        public static final double CATCH_HANG_GEAR_RATIO = 1.0;

        // TODO : Need to be Tuned.
        public static final SimpleMotorFeedforward HANG_FEED_FORWARD = new SimpleMotorFeedforward(0, 0, 0);
        public static final ArmFeedforward CATCH_HANG_FEED_FORWARD = new ArmFeedforward(0, 0, 0,0);

        public static final PIDConfig HANG_FEEDBACK = new PIDConfig(0, 0, 0);
        public static final PIDConfig CATCH_HANG_FEEDBACK = new PIDConfig(0, 0, 0);

        static {
            CATCH_HANG_CONFIG.encoder.positionConversionFactor(CATCH_HANG_GEAR_RATIO);
            CATCH_HANG_CONFIG.encoder.velocityConversionFactor(CATCH_HANG_GEAR_RATIO / 60);

            CATCH_HANG_CONFIG.idleMode(IdleMode.kBrake).smartCurrentLimit(40).voltageCompensation(12);
            CATCH_HANG_CONFIG.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // Set PID values for elevatorPosition control
                    // TODO: Need To be Tuned.
                    .apply(HANG_FEEDBACK.createSparkMaxConfig())
                    .iZone(0)
                    .outputRange(-1, 1)
                    .maxMotion
                    // Set MAXMotion parameters for elevatorPosition control
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

            public static final double GRAB_WRIST_GEAR_RATIO = 1.0 / 75.0;
            public static final double GRAB_INTAKE_GEAR_RATIO = 1.0 / 5;
            

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
                        // Set PID values for elevatorPosition control
                        .apply(GRAB_INTAKE_FEEDBACK.createSparkMaxConfig())
                        .iZone(0)
                        .outputRange(-1, 1)
                        .maxMotion
                        // Set MAXMotion parameters for elevatorPosition control
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
                        // Set PID values for elevatorPosition control
                        // TODO: Need To be Tuned.
                        .apply(GRAB_WRIST_FEEDBACK.createSparkMaxConfig())
                        .iZone(0)
                        .outputRange(-1, 1)
                        .maxMotion
                        // Set MAXMotion parameters for elevatorPosition control
                        .maxVelocity(2000)
                        .maxAcceleration(10000)
                        .allowedClosedLoopError(0.25);
            }


        }

        public static final class VisionConstants {
            // TODO : Need to be Tuned.
            // public static final PIDConfig VISION_FEEDBACK = new PIDConfig(0, 0, 0);
            // public static final SimpleMotorFeedforward VISION_FEED_FORWARD = new SimpleMotorFeedforward(0, 0, 0);

            public static final PIDController ALGAE_VISION_FEEDBACK = new PIDController(0, 0, 0);
            public static final double ALGAE_VISION_AIM_TOLERANCE = 0;
            public static final double ALGAE_VISION_AIM_INTEGRATOR_RANGE = 0.5;
            

            
        }






    }







}
    
    

