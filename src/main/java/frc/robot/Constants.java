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
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.config.PIDConstants;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.lib.control.FeedForward;
import frc.lib.control.PIDConfig;
import frc.lib.control.FeedForward.GravityType;
import frc.lib.util.PolynomialRegression;
import frc.robot.generated.TunerConstants;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

public class Constants {

    public enum Mode {
        SIM,
        COMP,
        REPLAY
    }

    public static enum ScoreState {
        L1(0.05, 0.0),
        L2(0.25, -45.0),//TODO CHANGE BACK
        L3(0.65, -45.0),
        L4(1.28, -45.0),
        ALGAE_L2(0.33, -25.0), //todo: TEST
        ALGAE_L3(0.68, -20.0),
        NORMAL(0.05, 90.0),
        STATION(0.14, 45.0),
        NET(1.285,60.0),
        ALGAE_NORMAL(0.15,90.0),
        AUTO_STATION(0.095,45.0);

        public double elevatorPosition;
        public double armPosition;
        private ScoreState(double elevatorPosition,double armPosition) {
            this.elevatorPosition = elevatorPosition;
            this.armPosition = armPosition;
        }
    }

    public static enum RobotState {
        PREP_L1,
        SCORE_L1,
        PREP_L2,
        SCORE_L2,
        PREP_L3,
        SCORE_L3,
        PREP_L4,
        SCORE_L4,
        NORMAL,
        PREP_STATION,
        SCORE_STATION,
        PREP_NET,
        SCORE_NET,
        HANG,
        PREP_L2_ALGAE,
        PREP_L3_ALGAE,
        EJECT_L2_ALGAE,
        EJECT_L3_ALGAE,
        PREP_PROCESSOR,
        SCORE_PROCESSOR,
        RESET,
        AUTO_STATION


    }

    public static enum TargetState {
        PREP_L1,
        PREP_L2,
        PREP_L3,
        PREP_L4,
        NORMAL,
        PREP_STATION,
        PREP_PROCESSOR,
        PREP_ALGAE_L2,
        PREP_ALGAE_L3,
        HANG,
        RESET,
        PREP_NET,
        AUTO_STATION

    }

    public static final class SwerveConstants {
        public static final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // m/s
        public static final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
    }

    public static final class AutoConstants {
        public static final PIDConstants TRANSLATION_PID = new PIDConstants(8.5, 0, 0);
        public static final PIDConstants ROTATION_PID = new PIDConstants(2.131, 0, 0);
    }

    public static final class PhotonAprilTagConstants {

        public static final double REEF_LEFT_PIPELINE = 0;
        public static final double REEF_RIGHT_PIPELINE = 1;

        public static double VISION_STD_DEV_X = 0.5;
        public static double VISION_STD_DEV_Y = 0.5;
        public static double VISION_STD_DEV_THETA = 99999999;

        public static final Matrix<N3, N1> visionStdMatrix =
                VecBuilder.fill(VISION_STD_DEV_X, VISION_STD_DEV_Y, VISION_STD_DEV_THETA);
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);

        public static final AprilTagFieldLayout TAG_FIELD_LAYOUT =
            AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

        public static final Transform3d[] REEF_CAMERA_OFFSET = new Transform3d[]{
            // Reef Left : 0 , Reef Right : 1
            new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0)),
            new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0))
        };

        public static final PolynomialRegression xyStdDevModel =
        new PolynomialRegression(
            new double[] {
                0.752358, 1.016358, 1.296358, 1.574358, 1.913358, 2.184358, 2.493358, 2.758358,
                3.223358, 4.093358, 4.726358
            },
            new double[] {0.005, 0.0135, 0.016, 0.038, 0.0515, 0.0925, 0.12, 0.14, 0.17, 0.27, 0.38},
            2);
        public static final PolynomialRegression thetaStdDevModel =
            new PolynomialRegression(
                new double[] {
                    0.752358, 1.016358, 1.296358, 1.574358, 1.913358, 2.184358, 2.493358, 2.758358,
                    3.223358, 4.093358, 4.726358
                },
                new double[] {0.008, 0.027, 0.015, 0.044, 0.04, 0.078, 0.049, 0.027, 0.059, 0.029, 0.068},
            1);
        
    }

    public static final class CoralConstants {

        public static final double CORAL_INTAKE_GEAR_RATIO = 4.0;
        public static final double CORAL_WRIST_LENGTH = 34.176;
        public static final double CORAL_WRIST_GEAR_RATIO = 225.0/4.0;

        // TODO : Need to be Tuned.
        public static final PIDConfig CORAL_INTAKE_FEEDBACK = new PIDConfig(0.11, 0, 0.0001,0.0292);
        public static final PIDConfig CORAL_WRIST_FEEDBACK = new PIDConfig(102.279, 0.031, 0.23262);

        public static final double CORAL_WRIST_KS = 0.60946/12;
        public static final double CORAL_WRIST_KV = 4.7417/12;
        public static final double CORAL_WRIST_KA = 0.67673/12;
        public static final double CORAL_WRIST_KG = 0.59022/12;


        public static final FeedForward CORAL_INTAKE_FF = FeedForward.builder()
            .kS(0.01).kV(0.04).kA(0.0005).build();

        public static final FeedForward CORAL_WRIST_FF = FeedForward.builder()
            .kS(1.00946/12.0).kV(5.6417/12.0).kA(0.67673/12.0).kG(0.79022/12.0).build();
        
        public static final double CORAL_INTAKE_VELOCITY = 30.0;
        
        public static final double CORAL_WRIST_FORWARD_SOFT_LIMIT = 90.0;
        public static final double CORAL_WRIST_REVERSE_SOFT_LIMIT = -54.0;

        public static final double CORAL_ENCODER_OFFSET = -0.06103515625;
        public static final double CORAL_WRIST_MAX_VELOCITY = 180;
        public static final double CORAL_WRIST_MAX_ACCEL = 60.0;
        public static final double CORAL_WRIST_MAX_JERK = 500.0;


        

        public static final CANcoderConfiguration CORAL_WRIST_ENCODER_CONFIG = new CANcoderConfiguration()
        .withMagnetSensor(new MagnetSensorConfigs()
            .withMagnetOffset(CORAL_ENCODER_OFFSET)
            .withAbsoluteSensorDiscontinuityPoint(0.5)
            .withSensorDirection(SensorDirectionValue.Clockwise_Positive));
        
            
        public static final TalonFXConfiguration CORAL_INTAKE_CONFIG = new TalonFXConfiguration() 
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimitEnable(true)
                    .withStatorCurrentLimit(100))
            .withVoltage(new VoltageConfigs()
                    .withPeakForwardVoltage(12)
                    .withPeakReverseVoltage(-12))
            .withFeedback(new FeedbackConfigs()
                    //.withFeedbackRemoteSensorID(CORAL_WRIST_ID.getDeviceNumber())
                    //.withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                    .withSensorToMechanismRatio(CORAL_INTAKE_GEAR_RATIO)
                    .withRotorToSensorRatio(1.0))
                    .withClosedLoopGeneral(new ClosedLoopGeneralConfigs()
                    .withContinuousWrap(true))
                    
                    //.withContinuousWrap(true))
            .withMotorOutput(new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(Brake))

            .withSlot0(new Slot0Configs()
                    .withKP(CORAL_INTAKE_FEEDBACK.P)
                    .withKI(CORAL_INTAKE_FEEDBACK.I)
                    .withKD(CORAL_INTAKE_FEEDBACK.D)
                    .withKS(CORAL_INTAKE_FF.getKS())
                    .withKV(CORAL_INTAKE_FF.getKV())
                    .withKA(CORAL_INTAKE_FF.getKA()));
            
        
        public static final TalonFXConfiguration CORAL_WRISTCONFIG = new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimitEnable(true)
                    .withStatorCurrentLimit(100))
            .withVoltage(new VoltageConfigs()
                    .withPeakForwardVoltage(12)
                    .withPeakReverseVoltage(-12))
            .withFeedback(new FeedbackConfigs()
                    //.withFeedbackRemoteSensorID(CORAL_WRIST_ID.getDeviceNumber())
                    //.withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                    .withSensorToMechanismRatio(1.0)
                    .withRotorToSensorRatio(CORAL_WRIST_GEAR_RATIO))
                    .withClosedLoopGeneral(new ClosedLoopGeneralConfigs())
                    
                    //.withContinuousWrap(true))
            .withMotorOutput(new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(Brake))

            .withSlot0(new Slot0Configs()
                    .withKP(CORAL_WRIST_FEEDBACK.P)
                    .withKI(CORAL_WRIST_FEEDBACK.I)
                    .withKD(CORAL_WRIST_FEEDBACK.D)
                    .withGravityType(GravityTypeValue.Arm_Cosine)
                    .withKS(CORAL_WRIST_FF.getKS())
                    .withKV(CORAL_WRIST_FF.getKV())
                    .withKA(CORAL_WRIST_FF.getKA())
                    .withKG(CORAL_WRIST_FF.getKG()))
            .withMotionMagic(new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(CORAL_WRIST_MAX_VELOCITY)
                    .withMotionMagicAcceleration(CORAL_WRIST_MAX_ACCEL)
                    .withMotionMagicJerk(CORAL_WRIST_MAX_JERK)
                    )
            .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
                    .withForwardSoftLimitEnable(true)
                    .withForwardSoftLimitThreshold(Units.degreesToRotations(CORAL_WRIST_FORWARD_SOFT_LIMIT))
                    .withReverseSoftLimitEnable(true)
                    .withReverseSoftLimitThreshold(Units.degreesToRotations(CORAL_WRIST_REVERSE_SOFT_LIMIT)));

    


        
    }


    public static final class ElevatorConstants {

       

        public static final double ELEVATOR_LENGTH = 0.06*2;
        public static final double ELEVATOR_GEAR_RATIO = 75;

        // TODO : Need to be Tuned.
        

        public static final double ELEVATOR_MAX_LENGTH = 1.287;
        public static final double ELEVATOR_MIN_LENGTH = 0.0;


        public static final FeedForward ELEVATOR_FEEDFORWARD = FeedForward.builder()
            .kS(1.36859/12.0).kV(12.923/12.0).kA(1.8629/12.0).kG(1.43059/12.0).gravityType(GravityType.kElevator).build();

        public static final double ELEVATOR_DEADZONE_DISTANCE = 0.015;
        public static final double ELEVAROR_MAX_VELOCITY = 300.0;
        public static final double ELEVAROR_MAX_ACCEL = 150.0;
        public static final double ELEVATOR_MAX_JERK = 1600.0;

        public static final double WITH_ZERO_TIMEOUT = 3.0;
        public static final double ZEROED_VOLTAGE = (-2.0);
        public static final double ZEROED_VELOCITY = -0.02;



        public static final PIDConfig ELEVATOR_FEEDBACK = new PIDConfig(109.629, 0, 0.16814);

        public static final TalonFXConfiguration ELEVATOR_CONFIG_L = new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimitEnable(true)
                    .withStatorCurrentLimit(120))
                .withVoltage(new VoltageConfigs()
                    .withPeakForwardVoltage(12)
                    .withPeakReverseVoltage(-12))
                .withFeedback(new FeedbackConfigs()
                    .withSensorToMechanismRatio(ELEVATOR_GEAR_RATIO))
                    .withClosedLoopGeneral(new ClosedLoopGeneralConfigs())
                    // .withContinuousWrap(false))
                .withMotorOutput(new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(Brake))

                .withSlot0(new Slot0Configs()
                    .withKP(ELEVATOR_FEEDBACK.P)
                    .withKI(ELEVATOR_FEEDBACK.I)
                    .withKD(ELEVATOR_FEEDBACK.D)
                    .withGravityType(GravityTypeValue.Elevator_Static)
                    .withKG(ELEVATOR_FEEDFORWARD.getKG())
                    .withKS(ELEVATOR_FEEDFORWARD.getKS())
                    .withKV(ELEVATOR_FEEDFORWARD.getKV())
                    .withKA(ELEVATOR_FEEDFORWARD.getKA()))

                .withMotionMagic(
                    new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(ELEVAROR_MAX_VELOCITY)
                    .withMotionMagicAcceleration(ELEVAROR_MAX_ACCEL)
                    .withMotionMagicJerk(ELEVATOR_MAX_JERK)
                    )
                .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
                    .withForwardSoftLimitEnable(true)
                    .withForwardSoftLimitThreshold(ELEVATOR_MAX_LENGTH)
                    .withReverseSoftLimitEnable(false)
                    .withReverseSoftLimitThreshold(ELEVATOR_MIN_LENGTH)
                );
            
            public static final TalonFXConfiguration ELEVATOR_CONFIG_R = new TalonFXConfiguration()
                .withCurrentLimits(
                    new CurrentLimitsConfigs()
                        .withStatorCurrentLimitEnable(true)
                        .withStatorCurrentLimit(120))
                    .withVoltage(new VoltageConfigs()
                        .withPeakForwardVoltage(12)
                        .withPeakReverseVoltage(-12))
                    .withFeedback(new FeedbackConfigs()
                        .withSensorToMechanismRatio(ELEVATOR_GEAR_RATIO))
                        .withClosedLoopGeneral(new ClosedLoopGeneralConfigs())
                        // .withContinuousWrap(false))
                    .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(InvertedValue.Clockwise_Positive)
                        .withNeutralMode(Brake))
    
                    .withSlot0(new Slot0Configs()
                        .withKP(ELEVATOR_FEEDBACK.P)
                        .withKI(ELEVATOR_FEEDBACK.I)
                        .withKD(ELEVATOR_FEEDBACK.D)
                        .withGravityType(GravityTypeValue.Elevator_Static)
                        .withKG(ELEVATOR_FEEDFORWARD.getKG())
                        .withKS(ELEVATOR_FEEDFORWARD.getKS())
                        .withKV(ELEVATOR_FEEDFORWARD.getKV())
                        .withKA(ELEVATOR_FEEDFORWARD.getKA()))
                    .withMotionMagic(new MotionMagicConfigs()
                        .withMotionMagicCruiseVelocity(ELEVAROR_MAX_VELOCITY)
                        .withMotionMagicAcceleration(ELEVAROR_MAX_ACCEL)
                        .withMotionMagicJerk(ELEVATOR_MAX_JERK)
                        )
                    .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
                        .withForwardSoftLimitEnable(true)
                        .withForwardSoftLimitThreshold(ELEVATOR_MAX_LENGTH)
                        .withReverseSoftLimitEnable(false)
                        .withReverseSoftLimitThreshold(ELEVATOR_MIN_LENGTH)
                    );


        //TODO : Need to be Tuned. & add degrees(?)

    }

    public static final class HangConstants {
        public static final SparkFlexConfig CATCH_HANG_CONFIG = new SparkFlexConfig();
        public static final double HANG_LENGTH = 34.176;
        public static final double HANG_GEAR_RATIO = 45.0;
        public static final double CATCH_HANG_GEAR_RATIO = 1.0;
        

        // TODO : Need to be Tuned.
        //public static final SimpleMotorFeedforward HANG_FEED_FORWARD = new SimpleMotorFeedforward(0, 0, 0);
        //public static final ArmFeedforward CATCH_HANG_FEED_FORWARD = new ArmFeedforward(0, 0, 0,0);

        public static final PIDConfig HANG_FEEDBACK = new PIDConfig(0.088066, 0, 0);
        public static final PIDConfig CATCH_HANG_FEEDBACK = new PIDConfig(0.0105, 0, 0.0001,0.0105);

    

        public static final FeedForward HANG_FEED_FORWARD = FeedForward.builder()
            .kS(0.002259/12.0).kV(5.2514/12.0).kA(1.2691/12.0).kG(0.07716/12.0).gravityType(GravityType.kArm).build();

        public static final double CATCH_HANG_INTAKE_VELOCITY = 10.0;

        public static final double HANG_FORWARD_LIMIT = 0.280518;
        public static final double HANG_REVERSE_LIMIT = -0.013184;

        public static final double HANG_ENCODER_OFFSET = -0.245117; 
        

        static {
            CATCH_HANG_CONFIG.encoder.positionConversionFactor(CATCH_HANG_GEAR_RATIO);
            CATCH_HANG_CONFIG.encoder.velocityConversionFactor(CATCH_HANG_GEAR_RATIO / 60);

            CATCH_HANG_CONFIG.idleMode(IdleMode.kCoast).smartCurrentLimit(60).voltageCompensation(12);
            CATCH_HANG_CONFIG.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // Set PID values for elevatorPosition control
                    // TODO: Need To be Tuned.
                    .apply(CATCH_HANG_FEEDBACK.createSparkMaxConfig())
                    .iZone(0)
                    .outputRange(-1, 1); 
                    
                    // Set MAXMotion parameters for elevatorPosition control
                    
                    
        }
        public static final CANcoderConfiguration HANG_ENCODER_CONFIG = new CANcoderConfiguration()
            .withMagnetSensor(new MagnetSensorConfigs()
                    .withMagnetOffset(HANG_ENCODER_OFFSET)
                    .withAbsoluteSensorDiscontinuityPoint(0.5)
                    .withSensorDirection(SensorDirectionValue.Clockwise_Positive));

        public static final TalonFXConfiguration HANG_CONFIG = new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(140.0)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(140.0)
                    .withSupplyCurrentLimitEnable(true))
                       
            .withVoltage(new VoltageConfigs()
                    .withPeakForwardVoltage(12)
                    .withPeakReverseVoltage(-12))
            .withFeedback(new FeedbackConfigs()
                    .withSensorToMechanismRatio(HANG_GEAR_RATIO)
                    .withRotorToSensorRatio(1.0))
                    
                    .withClosedLoopGeneral(new ClosedLoopGeneralConfigs()
                    .withContinuousWrap(false))
            .withMotorOutput(new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(Brake))

            .withSlot0(new Slot0Configs()
                    .withKP(HANG_FEEDBACK.P)
                    .withKI(HANG_FEEDBACK.I)
                    .withKD(HANG_FEEDBACK.D)
                    .withGravityType(GravityTypeValue.Arm_Cosine)
                    .withKS(HANG_FEED_FORWARD.getKS())
                    .withKV(HANG_FEED_FORWARD.getKV())
                    .withKA(HANG_FEED_FORWARD.getKA())
                    .withKG(HANG_FEED_FORWARD.getKG()))
            .withSlot1(new Slot1Configs()
                    .withKP(0.5)
                    .withKI(0)
                    .withKS(0.44)
                    .withKV(0.6))
            
            .withMotionMagic(new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(15.0)
                    .withMotionMagicAcceleration(30.0)
            )
            .withTorqueCurrent(new TorqueCurrentConfigs()
                    .withPeakForwardTorqueCurrent(800)
                    .withPeakReverseTorqueCurrent(-800)
            )
            .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
                    .withForwardSoftLimitEnable(false)
                    .withForwardSoftLimitThreshold(HANG_FORWARD_LIMIT)
                    .withReverseSoftLimitEnable(false)
                    .withReverseSoftLimitThreshold(HANG_REVERSE_LIMIT));

        
        

        public static final class GrabConstants {

            public static final SparkFlexConfig GRAB_WRIST_CONFIG = new SparkFlexConfig();
            public static final SparkFlexConfig GRAB_INTAKE_CONFIG = new SparkFlexConfig();

            public static final double GRAB_WRIST_GEAR_RATIO = 1.0 / 100.0;
            public static final double GRAB_INTAKE_GEAR_RATIO = 1.0 / 3.0;

            public static final double GRAB_KS = 0.67881/12;
            public static final double GRAB_KV = 11.6459/12;;
            public static final double GRAB_KA = 2.363/12;
            public static final double GRAB_KG = 0.98041/12;


            public static final double GRAB_INTAKE_VELOCITY = -15.0;
            public static final double GRAB_REVERSE_VELOCITY = 20.0;

            

            // TODO : Need to be Tuned.
            //public static final SimpleMotorFeedforward GRAB_INTAKE_FEED_FORWARD = new SimpleMotorFeedforward(0, 0, 0);
            public static final ArmFeedforward GRAB_WRIST_FEED_FORWARD = new ArmFeedforward(GRAB_KS, GRAB_KG, GRAB_KV,GRAB_KA);

            public static final PIDConfig GRAB_INTAKE_FEEDBACK = new PIDConfig(0.0005, 0, 0.00001,0.028);
            public static final PIDConfig GRAB_WRIST_FEEDBACK = new PIDConfig(0.019624, 0, 0.0011853);

            public static final double GRAB_WRIST_OFFSET_TOZERO = 0.180*360.0;

            public static final double GRAB_WRIST_OFFSET = 0.5570989;            ;

         


            static {
                GRAB_INTAKE_CONFIG.encoder.positionConversionFactor(GRAB_INTAKE_GEAR_RATIO);
                GRAB_INTAKE_CONFIG.encoder.velocityConversionFactor(GRAB_INTAKE_GEAR_RATIO /60.0);

                GRAB_INTAKE_CONFIG.idleMode(IdleMode.kBrake).smartCurrentLimit(40).voltageCompensation(12);
                GRAB_INTAKE_CONFIG.closedLoop
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        // Set PID values for elevatorPosition control
                        .apply(GRAB_INTAKE_FEEDBACK.createSparkMaxConfig())
                        .iZone(0)
                        .outputRange(-1, 1);
            }
            static {
                GRAB_WRIST_CONFIG.encoder.positionConversionFactor(GRAB_WRIST_GEAR_RATIO);
                GRAB_WRIST_CONFIG.encoder.velocityConversionFactor((GRAB_WRIST_GEAR_RATIO/60.0));
                //GRAB_WRIST_CONFIG.encoder.positionConversionFactor(360);
                GRAB_WRIST_CONFIG.absoluteEncoder.positionConversionFactor(1.0);
                GRAB_WRIST_CONFIG.absoluteEncoder.velocityConversionFactor(1.0/60.0);
                GRAB_WRIST_CONFIG.absoluteEncoder.zeroOffset(GRAB_WRIST_OFFSET);
                GRAB_WRIST_CONFIG.absoluteEncoder.inverted(true);

                GRAB_WRIST_CONFIG.idleMode(IdleMode.kBrake).smartCurrentLimit(60).voltageCompensation(12).inverted(false)
                .softLimit
                    .forwardSoftLimit(0.256)
                    .forwardSoftLimitEnabled(true)
                    .reverseSoftLimit(0.016)
                    .reverseSoftLimitEnabled(true);
                GRAB_WRIST_CONFIG.closedLoop
                        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                        // TODO: Need To be Tuned.
                        .apply(GRAB_WRIST_FEEDBACK.createSparkMaxConfig())
                        .iZone(0)
                        .outputRange(-1, 1)
                        .maxMotion
                        .maxVelocity(1920)
                        .maxAcceleration(7620)
                        .allowedClosedLoopError(Units.degreesToRotations(5.0));
                

                
            }


        }
        

        public static final class VisionConstants {
            // TODO : Need to be Tuned.
            // public static final PIDConfig VISION_FEEDBACK = new PIDConfig(0, 0, 0);
            // public static final SimpleMotorFeedforward VISION_FEED_FORWARD = new SimpleMotorFeedforward(0, 0, 0);

            public static final PIDController ALGAE_VISION_FEEDBACK = new PIDController(0, 0, 0);
            public static final double ALGAE_VISION_AIM_TOLERANCE = 0;
            public static final double ALGAE_VISION_AIM_INTEGRATOR_RANGE = 0.5;

            public static final double MIN_AREA_FOR_MEGATAG = 0.4; //TODO: test values
    
            public static final double frontLLToRobotX = 0.0; //TODO: find values
            public static final double frontLLToRobotY = 0.0; //TODO: find values\
            public static final double frontLLToRobotZ = 0.0; //TODO: find values
            public static final double frontLLroll = 0.0; //TODO: find values
            public static final double frontLLpitch = 0.0; //TODO: find values
            public static final double frontLLyaw = 0.0; //TODO: find values
            public static final Rotation2d frontLLToRobotTheta = new Rotation2d();  //TODO: find values

            public static final double backLLToRobotX = 0.0; //TODO: find values
            public static final double backLLToRobotY = 0.0; //TODO: find values
            public static final double backLLToRobotZ = 0.0; //TODO: find values
            public static final double backLLroll = 0.0; //TODO: find values
            public static final double backLLpitch = 0.0; //TODO: find values
            public static final double backLLyaw = 0.0; //TODO: find values
            public static final Rotation2d backLLToRobotTheta = new Rotation2d(0.0); //TODO: find values

            public static final double upperLLToRobotX = 0.0; //TODO: find values
            public static final double upperLLToRobotY = 0.0; //TODO: find values
            public static final double upperLLToRobotZ = 0.0; //TODO: find values
            public static final double upperLLroll = 0.0; //TODO: find values
            public static final double upperLLpitch = 0.0; //TODO: find values
            public static final double upperLLyaw = 0.0; //TODO: find values
            public static final Rotation2d upperLLToRobotTheta = new Rotation2d(); //TODO: find values

            public static final Matrix<N3,N1> odometryStdDev = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(0.1)); //TODO: find values 
            public static final Matrix<N3,N1> visionStdDev = VecBuilder.fill(0.9, 0.9, Units.degreesToRadians(1.0)); //TODO: find values

            //Todo: get PID VALUES 
            public enum limelightStrafePID{
                KP(0.6),
                KI(0.0),
                KD(0.0);

                private final double value;

                private limelightStrafePID(double value){
                    this.value = value;
                }

                public double getValue(){
                    return value;
                }
            }


            
            public enum limelightRotationPID{
                KP(0.5),
                KI(0.0),
                KD(0.0);

                private final double value;

                private limelightRotationPID(double value){
                    this.value = value;
                }

                public double getValue(){
                    return value;
                }
            }


            
            public enum limelightTranslatePID{
                KP(0.3),
                KI(0.0),
                KD(0.0);

                private final double value;

                private limelightTranslatePID(double value){
                    this.value = value;
                }

                public double getValue(){
                    return value;
                }
            }
        }

        

    }

    public static final boolean disableHAL = false;
    public static final String CANdleID = null;








}
    
    

