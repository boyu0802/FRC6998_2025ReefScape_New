package frc.robot.commands.setcommand;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.Constants.ScoreState;
import frc.robot.subsystem.coral.CoralSubsystem;


public class SetCoralWristCommand extends InstantCommand{
    private final CoralSubsystem coralSubsystem;
    private ScoreState scoreState;

    public SetCoralWristCommand(Constants.ScoreState scoreState, CoralSubsystem coralSubsystem) {
        this.scoreState = scoreState;
        this.coralSubsystem = coralSubsystem;
        addRequirements(coralSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        coralSubsystem.setCoralWristPosition(scoreState);
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
    
    
}
