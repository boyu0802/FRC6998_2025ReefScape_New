package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class SetRumbleCommand extends Command {
    private CommandXboxController xboxController;
    private double rumbleTime = 0.5;
    private double rumblePower = 0.5;
    private Timer timer = new Timer();

    public SetRumbleCommand(CommandXboxController xboxController) {
        // setName("SetRumbleCommand");
        this.xboxController = xboxController;
    }

    @Override
    public void initialize() {
        timer.restart();
        xboxController.setRumble(RumbleType.kBothRumble, rumblePower);
        // System.out.println("[SetRumbleCommand] initialize called!");

    }

    @Override
    public void execute() {
        // System.out.println("[SetRumbleCommand] execute called!");

    }

    @Override
    public void end(boolean interrupted) {
        if(isFinished() || interrupted) {
            xboxController.setRumble(RumbleType.kBothRumble, 0.0);
        }
        // System.out.println("[SetRumbleCommand] end called!");
    }

    @Override
    public boolean isFinished() {
        return (timer.get() > rumbleTime);
    }
    
}
