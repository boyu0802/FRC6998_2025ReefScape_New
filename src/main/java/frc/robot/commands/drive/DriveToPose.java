package frc.robot.commands.drive;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.drive.CommandSwerveDrivetrain;
import frc.robot.subsystem.vision.VisionState;

public class DriveToPose extends Command {
    private final CommandSwerveDrivetrain drive;
    private final Supplier<Pose2d> targetPose;
    private final VisionState state;
    private final ProfiledPIDController xTranslationController = new ProfiledPIDController(0.07, 0, 0, new TrapezoidProfile.Constraints(3, 4));
    private final ProfiledPIDController yTranslationController = new ProfiledPIDController(0.07, 0, 0, new TrapezoidProfile.Constraints(3, 4));
    private final ProfiledPIDController rotController = new ProfiledPIDController(0.07, 0.0, 0.0, new TrapezoidProfile.Constraints(3, 4));
    private final SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric();


    public DriveToPose(CommandSwerveDrivetrain drive, VisionState state, Supplier<Pose2d> targetPose) {
         this.drive = drive;
        this.targetPose = targetPose;
        this.state = state;
        rotController.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(drive);

    }

    @Override
    public void initialize(){
    
    }

    @Override
    public void execute(){
        Pose2d currentPose = state.getLatestFieldToRobot();
        double yPoseError = targetPose.get().getY() - currentPose.getY();
        double xPoseError = targetPose.get().getX() - currentPose.getX();
        double thetaPoseError = targetPose.get().getRotation().getRadians() - currentPose.getRotation().getRadians();

        drive.setControl(
            driveRequest
                .withVelocityX(xTranslationController.calculate(xPoseError,0.0))
                .withVelocityY(yTranslationController.calculate(yPoseError, 0.0))
                .withRotationalRate(rotController.calculate(thetaPoseError, 0.0))
        );

    }

}
