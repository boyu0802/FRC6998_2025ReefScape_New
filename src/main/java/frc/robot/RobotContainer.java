// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ElevatorConstants.ScoreState;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystem.drive.CommandSwerveDrivetrain;
import frc.robot.subsystem.elevator.ElevatorSubsystem;
import frc.robot.subsystem.hang.HangSubsystem;
import frc.robot.subsystem.algae.GrabSubsystem;
import frc.robot.subsystem.coral.CoralSubsystem;

import static frc.robot.Constants.SwerveConstants.MaxSpeed;

import java.util.Optional;

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
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 6% deadband
            .withDriveRequestType(DriveRequestType.Velocity);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController testController = new CommandXboxController(1);

    private final CommandXboxController testController2 = new CommandXboxController(2);
    


    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final CoralSubsystem coralSubsystem = new CoralSubsystem();
    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

    private final GrabSubsystem grabSubsystem = new GrabSubsystem();
    //private final HangSubsystem hangSubsystem = new HangSubsystem();
    
    


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
            drive.withVelocityX(Math.copySign(joystick.getLeftY()*joystick.getLeftY(),-joystick.getLeftY()) * MaxSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(Math.copySign(joystick.getLeftX()*joystick.getLeftX(),-joystick.getLeftX()) * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate)
            ) // Drive counterclockwise with negative X (left)
    
        );

        joystick.rightBumper().whileTrue(drivetrain.applyRequest(() -> brake));


        //93.4 0.3629281
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
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        //drivetrain.registerTelemetry(logger::telemeterize);
        //logger.elevatorTelemetry(elevatorSubsystem);
        

        //elevatorMechanism2d.getRoot("Position",0,elevatorSubsystem.getElevatorPosition()).append(elevatorLigament2d);
        //elevatorLigament2d.setAngle(0);
        //elevatorLigament2d.setLength(elevatorSubsystem.getElevatorPosition());
        //SmartDashboard.putData("Elevator",elevatorMechanism2d);
        

        
        

        // TODO: test by controller. (change with different subsystems)
       

        testController.povUp().onTrue(coralSubsystem.collectCoralWithoutVision());
        testController.povDown().onTrue(coralSubsystem.collectAlgaeWithoutVision());
        testController.povRight().onTrue(coralSubsystem.outputCoralWithoutVision());
        testController.povLeft().onTrue(coralSubsystem.outputAlgaeWithoutVision());

        
        
        testController.a().whileTrue(coralSubsystem.wristToL1());
        testController.b().whileTrue(coralSubsystem.wristToL2());
        testController.y().whileTrue(coralSubsystem.wristToL3());
        testController.x().whileTrue(coralSubsystem.wristToL4());
        
    
        
        
        testController.leftBumper().onTrue(elevatorSubsystem.setL1());
        testController.rightBumper().onTrue(elevatorSubsystem.setL2());
        testController.start().onTrue(elevatorSubsystem.setL3());
        testController.back().onTrue(elevatorSubsystem.setL4());
        
        testController2.leftBumper().onTrue(grabSubsystem.setGrabto10deg());
        testController2.rightBumper().onTrue(grabSubsystem.setGrabto75deg());
        

        testController2.start().and(testController2.povUp()).whileTrue(grabSubsystem.sysid_wristDynamic(Direction.kForward));
        testController2.start().and(testController2.povDown()).whileTrue(grabSubsystem.sysid_wristDynamic(Direction.kReverse));
        testController2.start().and(testController2.povRight()).whileTrue(grabSubsystem.sysid_wristQuasistatic(Direction.kForward));
        testController2.start().and(testController2.povLeft()).whileTrue(grabSubsystem.sysid_wristQuasistatic(Direction.kReverse));



        
        SmartDashboard.putNumber("Battery", RobotController.getBatteryVoltage());

    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
    
}