package frc.robot.commands.zeroing;

import static frc.robot.Constants.ElevatorConstants.ZEROED_VELOCITY;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.elevator.ElevatorSubsystem;

public class ZeroElevatorCommand extends Command {

    private ElevatorSubsystem elevator;
    //private boolean isZeroed = false;
    private Timer timer = new Timer();
    
    public ZeroElevatorCommand(ElevatorSubsystem elevator) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.elevator = elevator;
        addRequirements(elevator);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        elevator.stopElevator();
        timer.restart();
        //isZeroed = elevator.hasZeroed;

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        elevator.setVoltage(-0.2);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        elevator.setSoftwareLimits(true, true);
        elevator.stopElevator();
        if(!interrupted) {
            elevator.resetPosition();
            
        }
    }
    

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        
        return (elevator.getElevatorLimit() || timer.hasElapsed(3.0));
    }

    
}
