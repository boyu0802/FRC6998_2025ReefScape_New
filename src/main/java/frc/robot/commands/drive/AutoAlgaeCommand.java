package frc.robot.commands.drive;


import static frc.robot.Constants.SwerveConstants.MaxAngularRate;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.drive.Swerve;
import frc.robot.subsystem.vision.AlgaeDetect;

public class AutoAlgaeCommand extends Command {

    private final AlgaeDetect algaeDetect;
    private final Swerve swerve;

    private final ProfiledPIDController thetaController = new ProfiledPIDController(0.01, 0, 0, new Constraints(MaxAngularRate, 6.28));


    //private final SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric();
    private final SwerveRequest.PointWheelsAt pointWheelsAt = new SwerveRequest.PointWheelsAt();

    private Pose2d targetPose;
    private Pose2d driveToPose;

    private Rotation2d targetRotation = new Rotation2d(0.1);
    protected boolean isFinished = false;


    public AutoAlgaeCommand(AlgaeDetect algaeDetect, Swerve swerve) {
        this.algaeDetect = algaeDetect;
        this.swerve = swerve;
        addRequirements(swerve);
        addRequirements(algaeDetect);
        thetaController.setTolerance(0.1);
        
    }


    @Override
    public void initialize() {
        isFinished = false;

    }


    @Override
    public void execute(){
        if(!algaeDetect.isTargetVisible()){
            isFinished = true;
            return;
        }
        targetRotation = Rotation2d.fromDegrees(-algaeDetect.getTargetYaw());
        pointWheelsAt.withModuleDirection(targetRotation);
        isFinished = true;

    }





    
}
