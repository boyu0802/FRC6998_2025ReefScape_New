package frc.robot.commands.drive;

import java.util.Map;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.limelight.LimelightHelpers;
import frc.robot.subsystem.drive.CommandSwerveDrivetrain;
import frc.robot.subsystem.vision.Limelight;
import frc.robot.subsystem.vision.VisionState;

public class DriveToPose extends Command {
        private boolean isFinished = false;
        private final boolean isLeft;
        private final CommandSwerveDrivetrain drive;
        private final VisionState state;
        private double targetId;
        private final ProfiledPIDController xTranslationController = new ProfiledPIDController(0.5, 0.000002, 0.0, new TrapezoidProfile.Constraints(4, 4));
        private final ProfiledPIDController yTranslationController = new ProfiledPIDController(0.42, 0.00000002, 0.00000000012, new TrapezoidProfile.Constraints(4, 4));
        private final ProfiledPIDController rotController = new ProfiledPIDController(0.5, 0.000001, 0.0, new TrapezoidProfile.Constraints(4, 4));
        private final SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric();
        private Pose2d targetPose;
        private Pose2d driveToPose;
        private final Limelight front = new Limelight("limelight-front");
        private final Map<Integer, Pose2d> RED_ALLIANCE_POSE = Map.ofEntries(
            Map.entry(-1,new Pose2d()),
            Map.entry(6, new Pose2d(0.0,0.0, new Rotation2d(0.0))), Map.entry(7, new Pose2d(0.0,0.0, new Rotation2d(0.0))),
            Map.entry(8, new Pose2d(0.0,0.0, new Rotation2d(0.0))), Map.entry(9, new Pose2d(0.0,0.0, new Rotation2d(0.0))),
            Map.entry(10, new Pose2d(0.0,0.0, new Rotation2d(0.0))), Map.entry(11, new Pose2d(0.0,0.0, new Rotation2d(0.0)))
        );
        private final Map<Integer, Pose2d> BLUE_ALLIANCE_POSE = Map.ofEntries(
            Map.entry(-1,new Pose2d()),
            Map.entry(17, new Pose2d(0.0,0.0, new Rotation2d(0.0))), Map.entry(18, new Pose2d(3.2,3.99, new Rotation2d(0.0))),
            Map.entry(19, new Pose2d(0.0,0.0, new Rotation2d(0.0))), Map.entry(20, new Pose2d(0.0,0.0, new Rotation2d(0.0))),
            Map.entry(21, new Pose2d(0.0,0.0, new Rotation2d(0.0))), Map.entry(22, new Pose2d(0.0,0.0, new Rotation2d(0.0)))
        );
        //todo: find poses for all the targets //{ 3.2, 4.14}   , {3.2 , 3.84}
        private final double DIST_BETWEEN_TAG_AND_REEF = 0.15;
    
        public DriveToPose(CommandSwerveDrivetrain drive, VisionState state, boolean isLeft){
            this.drive = drive;
            this.state = state;
            rotController.enableContinuousInput(-Math.PI, Math.PI);
            targetId = front.getID();
            targetPose = state.isRedAlliance() ? RED_ALLIANCE_POSE.get((int)targetId) : BLUE_ALLIANCE_POSE.get((int)targetId);
            this.isLeft = isLeft;
            if(isLeft){
                driveToPose = new Pose2d(targetPose.getX(),targetPose.getY()+DIST_BETWEEN_TAG_AND_REEF,targetPose.getRotation());
            }else{
                driveToPose = new Pose2d(targetPose.getX(),targetPose.getY()-DIST_BETWEEN_TAG_AND_REEF,targetPose.getRotation());
            }
            addRequirements(drive);
        }
    
        @Override
        public void initialize(){
        isFinished = false;
        xTranslationController.setTolerance(0.01);
        yTranslationController.setTolerance(0.05);
        rotController.setTolerance(0.2);

    }
    @Override
    public void execute(){
        if(targetId == -1){
            isFinished = true;
        }
        Pose2d currentPose = state.getLatestFieldToRobot();
        double yPoseError = driveToPose.getY() - currentPose.getY();
        double xPoseError = driveToPose.getX() - currentPose.getX();
        double thetaPoseError = driveToPose.getRotation().getRadians() - currentPose.getRotation().getRadians();
        
        drive.setControl(
            driveRequest
                .withVelocityX(-xTranslationController.calculate(xPoseError,0.0)*4)
                .withVelocityY(-yTranslationController.calculate(yPoseError, 0.0)*4)
                .withRotationalRate(-rotController.calculate(thetaPoseError, 0.0)*4)
        );
        if(xTranslationController.atGoal() && yTranslationController.atGoal() && rotController.atGoal()){
            isFinished = true;
        }



    }

    @Override
    public boolean isFinished(){
        return isFinished;
    }

    @Override
    public void end(boolean interrupted){

    }

}
