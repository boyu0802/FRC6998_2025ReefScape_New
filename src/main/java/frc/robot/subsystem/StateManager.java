package frc.robot.subsystem;

import frc.robot.Constants.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TargetState;
import frc.robot.subsystem.algae.GrabSubsystem;
import frc.robot.subsystem.coral.CoralSubsystem;
import frc.robot.subsystem.elevator.ElevatorSubsystem;

public class StateManager extends SubsystemBase {
    public static RobotState currentRobotState;
    public static TargetState currentTargetState;

    CoralSubsystem coralSubsystem;
    GrabSubsystem grabSubsystem;
    ElevatorSubsystem elevatorSubsystem;
    StateManager superStructure = this;


    public StateManager(CoralSubsystem coralSubsystem, GrabSubsystem grabSubsystem, ElevatorSubsystem elevatorSubsystem) {
        this.coralSubsystem = coralSubsystem;
        this.grabSubsystem = grabSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;

        currentRobotState = RobotState.RESET;
        currentTargetState = TargetState.RESET;
    }

    public void setRobotState(RobotState state) {
        currentRobotState = state;
    }
    public void setTargetState(TargetState state) {
        currentTargetState = state;
    }

    public RobotState getRobotState() {
        return currentRobotState;
    }
    public TargetState getTargetState() {
        return currentTargetState;
    }







    
    
}
