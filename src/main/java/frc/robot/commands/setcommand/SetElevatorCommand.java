package frc.robot.commands.setcommand;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystem.elevator.ElevatorSubsystem;

public class SetElevatorCommand extends Command {

    private final Constants.ScoreState scoreState;
    private final ElevatorSubsystem elevatorSubsystem;
    public SetElevatorCommand(Constants.ScoreState scoreState, ElevatorSubsystem elevatorSubsystem) {
        this.scoreState = scoreState;
        this.elevatorSubsystem = elevatorSubsystem;
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
