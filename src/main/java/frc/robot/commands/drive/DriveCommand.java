package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.CommandSwerveDrivetrain;
import static frc.robot.Constants.SwerveConstants.MaxSpeed;
import static frc.robot.Constants.SwerveConstants.MaxAngularRate;

public class DriveCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final double translationSupplier;
    private final double strafeSupplier;
    private final double rotationSupplier;
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.06).withRotationalDeadband(MaxAngularRate * 0.06) // Add a 6% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    public DriveCommand(CommandSwerveDrivetrain drivetrain, double translationSupplier, double strafeSupplier, double rotationSupplier) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.drivetrain = drivetrain;
        this.translationSupplier = translationSupplier;
        this.strafeSupplier = strafeSupplier;
        this.rotationSupplier = rotationSupplier;
        addRequirements(drivetrain);
        
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
       
        drivetrain.applyRequest(() ->
                drive.withVelocityX(translationSupplier * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(strafeSupplier * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(rotationSupplier * MaxAngularRate) // Drive counterclockwise with negative X (left)
            
        );
            
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    
}
