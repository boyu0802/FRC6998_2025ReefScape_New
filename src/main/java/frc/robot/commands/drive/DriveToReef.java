package frc.robot.commands.drive;
import java.util.Map;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.drive.CommandSwerveDrivetrain;
import frc.robot.subsystem.vision.Limelight;

// NOT USED

public class DriveToReef extends Command{
    public static final ProfiledPIDController translateXController = new ProfiledPIDController(0, 0, 0, null);
    public static final ProfiledPIDController translateYController = new ProfiledPIDController(0, 0, 0, null);
    public static final ProfiledPIDController rotateThetaController = new ProfiledPIDController(0,0,0,null);
    private final Map<Integer, Double> APRIL_TAG_ROTATIONS = Map.ofEntries(
        Map.entry(6, 300.0), Map.entry(7, 0.0), Map.entry(8, 60.0), Map.entry(9, 120.0), Map.entry(10, 180.0),
        Map.entry(11, 240.0), Map.entry(17, 240.0), Map.entry(18, 180.0), Map.entry(19, 120.0), Map.entry(20, 60.0),
        Map.entry(21, 0.0), Map.entry(22, 300.0)
    );

    private static final SwerveRequest.FieldCentricFacingAngle driveRequest = new SwerveRequest.FieldCentricFacingAngle();

    private double goalTx;
    private double goalTy;
    private Limelight limelight;
    private CommandSwerveDrivetrain drive;
    private double driveAngle;
    private boolean endEarly;
    private double angle;
    
    public DriveToReef(CommandSwerveDrivetrain drive, Limelight limelight, double goalTx, double goalTy){
        this.goalTx = goalTx;
        this.goalTy = goalTy;
        this.limelight = limelight;
        this.drive = drive;
        addRequirements(drive);
    }
    @Override
    public void initialize(){
        endEarly = false;
        angle = APRIL_TAG_ROTATIONS.get((int)limelight.getID());
    }

    @Override
    public void execute(){
        driveAngle = Math.abs(angle - 180);
        if(!limelight.getTv()) endEarly = true;

        double xVelocity = translateXController.calculate( limelight.getTY() ,goalTy);
        double yVelocity = translateYController.calculate(limelight.getTx(), goalTx);
        double rotVelocity = rotateThetaController.calculate(drive.getState().RawHeading.getDegrees(), driveAngle);

        Translation2d robotSpeed = new Translation2d(xVelocity,yVelocity);
        Rotation2d driveRotate = new Rotation2d(rotVelocity);
        robotSpeed.rotateBy(driveRotate);
        drive.setControl(driveRequest.withVelocityX(robotSpeed.getX()).withVelocityY(robotSpeed.getY()).withTargetDirection(driveRotate));

    }

    @Override
    public void end(boolean interrupted) {
        return;
    }

    @Override
    public boolean isFinished(){
        return endEarly;
    }


}
