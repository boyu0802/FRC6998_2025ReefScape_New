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
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ScoreState;
import frc.robot.Constants.TargetState;
import frc.robot.commands.CoralIntakeCommand;
import frc.robot.commands.ReefStatePosition;
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
    /* Setting up bindings for necessary control of the swerve drive platform */

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    
    

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.06).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 6% deadband
            .withDriveRequestType(DriveRequestType.Velocity);

    private final CommandXboxController m_driveController = new CommandXboxController(0);
    private final CommandXboxController m_operatorController = new CommandXboxController(1);

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
    StateManager stateManager = StateManager.getInstance(coralSubsystem, grabSubsystem, elevatorSubsystem,m_driveController);
    private SequentialCommandGroup currentCoralCommand = new SequentialCommandGroup();

    CoralIntakeCommand coralIntakeCommand = new CoralIntakeCommand(coralSubsystem);
    ReefStatePosition reefStatePosition = new ReefStatePosition(coralSubsystem, elevatorSubsystem, m_operatorController);
    private Command currentReefState = new SequentialCommandGroup();
    

    


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
            .withVelocityX(-m_driveController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(-m_driveController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-m_driveController.getRightX() * MaxAngularRate)
            ) // Drive counterclockwise with negative X (left)
    
        );

        

        m_driveController.leftTrigger(0.95).whileTrue(new DriveToPose(drivetrain, visionState, true,m_driveController));
        m_driveController.rightTrigger(0.95).whileTrue(new DriveToPose(drivetrain, visionState, false,m_driveController));
        m_driveController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        m_driveController.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-m_driveController.getLeftY(), -m_driveController.getLeftX()))
        ));
        m_driveController.x().whileTrue(drivetrain.applyRequest(() -> brake));
        m_driveController.y().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));


        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        m_driveController.back().and(m_driveController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        m_driveController.back().and(m_driveController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        m_driveController.start().and(m_driveController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        m_driveController.start().and(m_driveController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        

        drivetrain.registerTelemetry(logger::telemeterize);
        //logger.elevatorTelemetry(elevatorSubsystem);
        

        //elevatorMechanism2d.getRoot("Position",0,elevatorSubsystem.getElevatorPosition()).append(elevatorLigament2d);
        //elevatorLigament2d.setAngle(0);
        //elevatorLigament2d.setLength(elevatorSubsystem.getElevatorPosition());
        //SmartDashboard.putData("Elevator",elevatorMechanism2d);
        
        m_operatorController.povUp().onTrue(new InstantCommand(()->{
            currentCoralCommand = coralIntakeCommand.chooseCommand();
            currentCoralCommand.schedule();
        }));
        m_operatorController.povDown().onTrue(coralSubsystem.collectAlgaeWithoutVision());
        m_operatorController.povRight().onTrue(coralSubsystem.outputCoralWithoutVision());
        m_operatorController.povLeft().onTrue(coralSubsystem.outputAlgaeWithoutVision());
        

        // Reef Positioning.
        m_operatorController.a().onTrue(new InstantCommand(()->{
            currentReefState = stateManager.SetReefState(TargetState.PREP_L1);
            currentReefState.schedule();
            
        }));
        m_operatorController.b().onTrue(new InstantCommand(()->{
            currentReefState = stateManager.SetReefState(TargetState.PREP_L2);
            currentReefState.schedule();
        }));
        m_operatorController.y().onTrue(new InstantCommand(()->{
            currentReefState = stateManager.SetReefState(TargetState.PREP_L3);
            currentReefState.schedule();
        }));
        m_operatorController.x().onTrue(new InstantCommand(()->{
            currentReefState = stateManager.SetReefState(TargetState.PREP_L4);
            currentReefState.schedule();
        }));
        m_operatorController.leftBumper().onTrue(new InstantCommand(()->{
            currentReefState = stateManager.SetReefState(TargetState.PREP_STATION);
            currentReefState.schedule();
        }));
        m_operatorController.rightBumper().onTrue(new InstantCommand(()->{
            currentReefState = stateManager.SetReefState(TargetState.NORMAL);
            currentReefState.schedule();
        }));
        m_operatorController.start().onTrue(new InstantCommand(()->{
            currentReefState = stateManager.SetReefState(TargetState.PREP_ALGAE_L2);
            currentReefState.schedule();
        }));
        m_operatorController.back().onTrue(new InstantCommand(()->{
            currentReefState = stateManager.SetReefState(TargetState.PREP_ALGAE_L3);
            currentReefState.schedule();
        }));
        
        
        
        

        
        

        m_driveController.povUp().onTrue(grabSubsystem.setGrabto10deg());
        m_driveController.povDown().onTrue(grabSubsystem.setGrabto75deg());
        m_driveController.povLeft().onTrue(grabSubsystem.collectWithoutVision());
        m_driveController.povRight().onTrue(grabSubsystem.reverseWithoutVision());

        
        
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