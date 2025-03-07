package frc.robot.commands.setcommand;

import static frc.robot.Constants.ElevatorConstants.ELEVATOR_MAX_LENGTH;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystem.coral.CoralSubsystem;
import frc.robot.subsystem.elevator.ElevatorSubsystem;
import lombok.Data;
import lombok.Getter;
import lombok.Setter;


public class ManualElevatorCommand extends Command {
    private final ElevatorSubsystem elevator;

    @Getter @Setter private double offset = 0.0;
    private final DoubleSupplier supplier;
    private double elevatorPosition = 0.0;
    private final Timer timer = new Timer();


    private ManualElevatorCommand(ElevatorSubsystem elevator,double offset,DoubleSupplier supplier) {
        this.elevator = elevator;
        addRequirements(elevator);
        this.offset = offset;
        this.supplier = supplier;
    }


    @Override
    public void initialize(){
        //timer.reset();
        elevatorPosition  = elevator.getLeftElevatorPosition();
    }

    @Override 
    public void execute(){

        if(supplier.getAsDouble() > 0.7) {
            offset+=0.01;
            timer.start();
            if (timer.get() >= 0.2 && elevatorPosition+offset <= ELEVATOR_MAX_LENGTH ) {
                offset+=0.01;
                timer.restart();
            }
        }
        else if(supplier.getAsDouble() < -0.7) {
            offset-=0.01;
            timer.start();
            if (timer.get() >= 0.2 && elevatorPosition+offset <= ELEVATOR_MAX_LENGTH ) {
                offset-=0.01;
                timer.restart();
            }
        }
        //elevator.

        SmartDashboard.putNumber("offset", getOffset());

    }

    
}
