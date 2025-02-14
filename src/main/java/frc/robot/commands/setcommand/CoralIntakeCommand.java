package frc.robot.commands.setcommand;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystem.coral.CoralSubsystem;

public class CoralIntakeCommand {

    private final CoralSubsystem coralSubsystem;
    private boolean isWorked = false;
    private Timer timer = new Timer();

    public CoralIntakeCommand(CoralSubsystem coralSubsystem) {
        this.coralSubsystem = coralSubsystem;
        //addRequirements(coralSubsystem);
    }

    private void setWorked(boolean worked){
        isWorked = worked;
    }

    /*
     * fixed the logic error in the chooseCommand method
     * isWorked = intake is working. If it is working, stop the intake
     */
    public SequentialCommandGroup chooseCommand(){
        SequentialCommandGroup newCommand;
        if(isWorked) {
            isWorked = false;
            newCommand = new SequentialCommandGroup(
                new InstantCommand(coralSubsystem::stopCoralIntake)
            );
        }
        else {
            isWorked = true;
            newCommand =  new SequentialCommandGroup(
                new InstantCommand(()-> coralSubsystem.setCoralIntakeVelocity(10)),
                new WaitUntilCommand(()->coralSubsystem.getCoralLimit()|| !isWorked),
                new InstantCommand(() -> coralSubsystem.stopCoralIntake()),
                new InstantCommand(()->setWorked(false))
            );
            
        }
        return newCommand;
    }
}
