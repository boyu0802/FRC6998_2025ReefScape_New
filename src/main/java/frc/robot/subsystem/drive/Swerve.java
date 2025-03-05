package frc.robot.subsystem.drive;

import static edu.wpi.first.units.Units.*;

import java.util.List;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.Field.ReefConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.subsystem.vision.VisionFieldPoseEstimate;
import frc.robot.subsystem.vision.VisionState;
import lombok.Getter;

import static frc.robot.Constants.AutoConstants.TRANSLATION_PID;
import static frc.robot.Constants.AutoConstants.ROTATION_PID;
/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class Swerve extends TunerSwerveDrivetrain implements Subsystem {
    private VisionState state;   
    
    private Field2d field = new Field2d();

    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    @Getter private boolean waypointsTransformed = false;
    //private final SwerveSetpointGenerator setpointGenerator;
    private SwerveSetpoint previousSetpoint;

    @Getter private Pose2d target = new Pose2d(0,0,new Rotation2d(0));

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            Seconds.of(5.0),        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineRotation;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
     * @param modules               Constants for each specific module
     */
    public Swerve(
        SwerveDrivetrainConstants drivetrainConstants,
        VisionState state,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        this.state = state;
        SmartDashboard.putData("poseEst",field);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
        initializeOtf();
        
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public Swerve(
        SwerveDrivetrainConstants drivetrainConstants,
        VisionState state,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        this.state = state;
        SmartDashboard.putData("poseEst",field);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
        initializeOtf();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry calculation
     *                                  in the form [x, y, theta], with units in meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision calculation
     *                                  in the form [x, y, theta], with units in meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public Swerve(
        SwerveDrivetrainConstants drivetrainConstants,
        VisionState state,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        this.state = state;
        SmartDashboard.putData("poseEst",field);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
        initializeOtf();
    }

    private void configureAutoBuilder() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                () -> state.getLatestFieldToRobot(),   // Supplier of current robot pose
                this::resetPose,         // Consumer for seeding pose against auto
                () -> state.getRobotRelativeSpeeds(), // Supplier of current robot speeds
                // Consumer of ChassisSpeeds and feedforwards to drive the robot
                (speeds, feedforwards) -> setControl(
                    m_pathApplyRobotSpeeds.withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),
                new PPHolonomicDriveController(
                    // PID constants for translation
                    TRANSLATION_PID,
                    // PID constants for rotation
                    ROTATION_PID
                ),
                config,
                // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this // Subsystem for requirements
            );
            new SwerveSetpointGenerator(config, Units.rotationsToRadians(10.0));
            
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());


        }

        

    }

    public void addVisionMeasurement(VisionFieldPoseEstimate visionFieldPoseEstimate) {
        addVisionMeasurement(
            visionFieldPoseEstimate.getVisionRobotPoseMeters(),
            Utils.fpgaToCurrentTime(visionFieldPoseEstimate.getTimestampSeconds()),
            visionFieldPoseEstimate.getVisionMeasurementStdDevs()
        );
    }

    

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * //@param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    public Command stop() {
        SwerveRequest.RobotCentric swerveRequest = new SwerveRequest.RobotCentric();

        return runOnce(
            () -> this.setControl(
                swerveRequest.withVelocityX(0.0).withVelocityY(0.0).withRotationalRate(0.0)
            )
        );
    }

    public Command goToPose(Pose2d pose) {
        return defer(
            () -> AutoBuilder.pathfindToPose(
                pose,
                new PathConstraints(
                    4.0,
                    3.0,
                    Units.degreesToRadians(540),
                    Units.degreesToRadians(720)
                )
            ).finallyDo((interrupted) -> stop())
        );
    }

    public Command pathToReef(boolean isLeft) {
        return defer(() -> {
            target = null;

            if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
                target = this.getState().Pose.nearest(
                    isLeft ? ReefConstants.LEFT_REEF_WAYPOINTS : ReefConstants.RIGHT_REEF_WAYPOINTS
                );
            } else {
                target = this.getState().Pose.nearest(
                    isLeft ? ReefConstants.LEFT_REEF_WAYPOINTS :ReefConstants.RIGHT_REEF_WAYPOINTS
                );
            }

            return goToPose(target).withTimeout(0.01).andThen(goToPose(target));
        });
    }

    public Command pathToStation() {
        return defer(() -> {
            Pose2d target = this.getState().Pose.nearest(ReefConstants.STATION_WAYPOINTS);

            return goToPose(target).withTimeout(0.01).andThen(goToPose(target));
        });
    }

    private void transformWaypointsForAlliance(List<Pose2d> waypoints) {
        final double FIELD_LENGTH = 17.55;
        final double X_OFFSET = 0.0;

        for (int i = 0; i < waypoints.size(); i++) {
            Pose2d bluePose = waypoints.get(i);
            waypoints.set(
                i,
                new Pose2d(
                    FIELD_LENGTH - bluePose.getX() + X_OFFSET,
                    bluePose.getY(),
                    Rotation2d.fromDegrees(180 - bluePose.getRotation().getDegrees())
                )
            );
        }
    }

    public void initializeOtf() {
        if ((DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) && !waypointsTransformed) {
            waypointsTransformed = true;

            transformWaypointsForAlliance(frc.robot.Field.ReefConstants.STATION_WAYPOINTS);
            transformWaypointsForAlliance(frc.robot.Field.ReefConstants.LEFT_REEF_WAYPOINTS);
            transformWaypointsForAlliance(frc.robot.Field.ReefConstants.RIGHT_REEF_WAYPOINTS);
        }
    }

    

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }

        state.addDriveMeasurement(
            Timer.getTimestamp(), 
            getPigeon2().getAngularVelocityZWorld().getValueAsDouble(), 
            getKinematics().toChassisSpeeds(getState().ModuleStates), 
            ChassisSpeeds.fromRobotRelativeSpeeds(getKinematics().toChassisSpeeds(getState().ModuleStates),
            getState().Pose.getRotation()),
            getState().Pose.getRotation().getDegrees());
        state.addOdometryMeasurement(Timer.getTimestamp(),getState().Pose);
        field.setRobotPose(getState().Pose);
        SmartDashboard.putString("pose est.",getState().Pose.toString());
        SmartDashboard.putBoolean("transformed",waypointsTransformed);
        

        

        
    }

    

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }
}