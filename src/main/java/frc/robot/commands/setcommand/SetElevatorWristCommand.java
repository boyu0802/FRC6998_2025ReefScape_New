package frc.robot.commands.setcommand;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystem.coral.CoralSubsystem;
import frc.robot.subsystem.elevator.ElevatorSubsystem;

public class SetElevatorWristCommand extends Command {
    private final Constants.ScoreState scoreState;
    private final ElevatorSubsystem elevatorSubsystem;
    private final CoralSubsystem coralSubsystem;
    public SetElevatorWristCommand(Constants.ScoreState scoreState, ElevatorSubsystem elevatorSubsystem,CoralSubsystem coralSubsystem) {
        this.scoreState = scoreState;
        this.elevatorSubsystem = elevatorSubsystem;
        this.coralSubsystem = coralSubsystem;
        addRequirements(elevatorSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        elevatorSubsystem.setElevatorPosition(scoreState);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(elevatorSubsystem.isAtWristPoint()) {
            coralSubsystem.setCoralWristPosition(scoreState);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.stopElevator();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return elevatorSubsystem.isAtSetpoint();
    }
    
}
