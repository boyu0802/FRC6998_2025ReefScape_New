// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ScoreState;
import frc.robot.Constants.TargetState;
import frc.robot.commands.CoralIntakeCommand;
import frc.robot.commands.SetElevatorCommand;
import frc.robot.commands.drive.DriveToPose;
import frc.robot.commands.zeroing.ZeroElevatorCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystem.drive.CommandSwerveDrivetrain;
import frc.robot.subsystem.elevator.ElevatorSubsystem;
import frc.robot.subsystem.hang.HangSubsystem;
import frc.robot.subsystem.vision.VisionFieldPoseEstimate;
import frc.robot.subsystem.vision.VisionState;
import frc.robot.subsystem.vision.VisionSubsystem;
import frc.robot.subsystem.StateManager;
import frc.robot.subsystem.algae.GrabSubsystem;
import frc.robot.subsystem.coral.CoralSubsystem;

import static frc.robot.Constants.SwerveConstants.MaxSpeed;

import java.util.Optional;
import java.util.function.Consumer;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Constants.SwerveConstants.MaxAngularRate;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    
    

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.06).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 6% deadband
            .withDriveRequestType(DriveRequestType.Velocity);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController testController = new 
    CommandXboxController(1);

    private final CommandXboxController testController2 = new CommandXboxController(2);
    private final CommandXboxController testController3 = new CommandXboxController(3);
    private final Consumer<VisionFieldPoseEstimate> visionFieldPoseEstimateConsumer = new Consumer<VisionFieldPoseEstimate>() {
        @Override
        public void accept(VisionFieldPoseEstimate visionFieldPoseEstimate) {
            drivetrain.addVisionMeasurement(visionFieldPoseEstimate);
            SmartDashboard.putNumber("vision X into drivetrain", visionFieldPoseEstimate.getVisionRobotPoseMeters().getX());
            SmartDashboard.putNumber("vision Y into drivetrain", visionFieldPoseEstimate.getVisionRobotPoseMeters().getY());
            
        }
    };

    private final VisionState visionState = new VisionState(visionFieldPoseEstimateConsumer);
    private final VisionSubsystem visionSubsystem = new VisionSubsystem(visionState);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain(visionState);
    private final CoralSubsystem coralSubsystem = new CoralSubsystem();
    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

    private final GrabSubsystem grabSubsystem = new GrabSubsystem();
    private final HangSubsystem hangSubsystem = new HangSubsystem();
    //private SetElevatorCommand setElevatorCommand = new SetElevatorCommand(ScoreState.L1,elevatorSubsystem);
    StateManager stateManager = StateManager.getInstance(coralSubsystem, grabSubsystem, elevatorSubsystem);
    private SequentialCommandGroup currentCoralCommand = new SequentialCommandGroup();

    CoralIntakeCommand coralIntakeCommand = new CoralIntakeCommand(coralSubsystem);


    


    //Optional to mirror the NetworkTables-logged data to a file on disk

    
    private final SendableChooser<Command> autoChooser;
    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("Example");
        SmartDashboard.putData("Auto Mode", autoChooser);
        //selectAuto();
        configureBindings();
        DataLogManager.start();

    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        
        elevatorSubsystem.updateTelemetry();
        drivetrain.setDefaultCommand(
          drivetrain.applyRequest(() ->
            drive
            .withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate)
            ) // Drive counterclockwise with negative X (left)
    
        );

        joystick.rightBumper().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));


        
        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));


        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        

        drivetrain.registerTelemetry(logger::telemeterize);
        //logger.elevatorTelemetry(elevatorSubsystem);
        

        //elevatorMechanism2d.getRoot("Position",0,elevatorSubsystem.getElevatorPosition()).append(elevatorLigament2d);
        //elevatorLigament2d.setAngle(0);
        //elevatorLigament2d.setLength(elevatorSubsystem.getElevatorPosition());
        //SmartDashboard.putData("Elevator",elevatorMechanism2d);
        

        
        

        // TODO: test by controller. (change with different subsystems)
       
        
        testController.povUp().onTrue(new InstantCommand(()->{
            currentCoralCommand = coralIntakeCommand.chooseCommand();
            currentCoralCommand.schedule();
        }));
        testController.povDown().onTrue(coralSubsystem.collectAlgaeWithoutVisipoknmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm mmmmmmmmmmmmmmmmm on());
        testController.povRight().onTrue(coralSubsystem.outputCoralWithoutVision());
        testController.povLeft().onTrue(coralSubsystem.outputAlgaeWithoutVision());
        



        testController.a().onTrue(stateManager.ElevatorSequence(TargetState.PREP_L1,stateManager.getRobotState()));
        testController.b().onTrue(stateManager.ElevatorSequence(TargetState.PREP_L2, stateManager.getRobotState()));
        testController.y().onTrue(stateManager.ElevatorSequence(TargetState.PREP_L3, stateManager.getRobotState()));
        testController.x().onTrue(stateManager.ElevatorSequence(TargetState.PREP_L4, stateManager.getRobotState()));
        testController.leftBumper().onTrue(stateManager.ElevatorSequence(TargetState.PREP_STATION, stateManager.getRobotState()));
        testController.rightBumper().onTrue(stateManager.ElevatorSequence(TargetState.NORMAL, stateManager.getRobotState()));
        testController.start().onTrue(stateManager.ElevatorSequence(TargetState.ALGAE_L2, stateManager.getRobotState()));
        testController.back().onTrue(stateManager.ElevatorSequence(TargetState.ALGAE_L3, stateManager.getRobotState()));
        

        joystick.povUp().onTrue(grabSubsystem.setGrabto10deg());
        joystick.povDown().onTrue(grabSubsystem.setGrabto75deg());
        joystick.povLeft().onTrue(grabSubsystem.collectWithoutVision());
        joystick.povRight().onTrue(grabSubsystem.reverseWithoutVision());

        
        
        testController2.leftBumper().onTrue(hangSubsystem.setHangto0deg());
        testController2.rightBumper().onTrue(hangSubsystem.setHangto90deg());
        testController2.axisGreaterThan(1, 0.5).onTrue(coralSubsystem.wristToNormal());
        testController2.povLeft().onTrue(hangSubsystem.catchHang());
        

        testController3.povUp().onTrue(elevatorSubsystem.increaseElevatorPositionCmd());
        testController3.povDown().onTrue(elevatorSubsystem.decreaseElevatorPositionCmd());

        
        SmartDashboard.putNumber("Battery", RobotController.getBatteryVoltage());

    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public Command zeroCommand() {
        return new ZeroElevatorCommand(elevatorSubsystem);
       
    }

    public boolean isZeroed() {
        return elevatorSubsystem.getElevatorLimit();
    }
    
}