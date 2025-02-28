package frc.robot.commands.setcommand;

import static frc.robot.Constants.HangConstants.HANG_FORWARD_LIMIT;
import static frc.robot.Constants.HangConstants.HANG_REVERSE_LIMIT;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystem.elevator.ElevatorSubsystem;
import frc.robot.subsystem.hang.HangSubsystem;

public class SetHangVelocityCommand extends Command {

    private final HangSubsystem hangSubsystem;
    private final double velocity;
    
    public SetHangVelocityCommand(HangSubsystem hangSubsystem,double velocity) {
        this.hangSubsystem = hangSubsystem;
        this.velocity = velocity;
        addRequirements(hangSubsystem);
    }
    

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if(velocity < 0 && hangSubsystem.gethangAbsoultePosition() < HANG_REVERSE_LIMIT){
            hangSubsystem.setHangVelocity(0);
        }
        else if( velocity > 0 && hangSubsystem.gethangAbsoultePosition() > HANG_FORWARD_LIMIT) {
            hangSubsystem.setHangVelocity(0);
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        hangSubsystem.setHangVelocity(velocity);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        hangSubsystem.setHangVelocity(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (velocity < 0 && hangSubsystem.gethangAbsoultePosition() < HANG_REVERSE_LIMIT) || (velocity > 0 && hangSubsystem.gethangAbsoultePosition() > HANG_FORWARD_LIMIT) ;
    }
}
    

