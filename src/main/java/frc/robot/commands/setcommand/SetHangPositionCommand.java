package frc.robot.commands.setcommand;

import static frc.robot.Constants.HangConstants.HANG_FORWARD_LIMIT;
import static frc.robot.Constants.HangConstants.HANG_REVERSE_LIMIT;
import static java.lang.Math.min;
import static java.lang.Math.max;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystem.hang.HangSubsystem;

public class SetHangPositionCommand extends Command {

    private final HangSubsystem hangSubsystem;
    private final double velocity;
    private final double position;
    
    
    public SetHangPositionCommand(HangSubsystem hangSubsystem,double velocity,double position) {
        this.hangSubsystem = hangSubsystem;
        this.velocity = velocity;
        this.position = position;
        addRequirements(hangSubsystem);
    }
    

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        double curPosition = hangSubsystem.gethangAbsoultePosition();
        if(velocity < 0 && curPosition < max(HANG_REVERSE_LIMIT,position)){
            hangSubsystem.setHangVelocity(0);
        }
        else if( velocity > 0 && curPosition > min(HANG_FORWARD_LIMIT, position)) {
            hangSubsystem.setHangVelocity(0);
        }
        else {
            hangSubsystem.setHangVelocity(velocity);
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        hangSubsystem.setHangVelocity(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (velocity < 0 && hangSubsystem.gethangAbsoultePosition() < max(HANG_REVERSE_LIMIT,position)) || (velocity > 0 && hangSubsystem.gethangAbsoultePosition() > min(HANG_FORWARD_LIMIT, position));
    }
}
    

