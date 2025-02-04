package frc.robot.commands.zeroing;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.elevator.ElevatorSubsystem;

public class ZeroElevatorCommand extends Command {

    private ElevatorSubsystem elevator;
    private boolean isZeroed = false;
    
    public ZeroElevatorCommand() {
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
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
