package frc.robot.subsystem;

import frc.robot.Constants.RobotState;
import frc.robot.Constants.ScoreState;

import java.lang.annotation.Target;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.TargetState;
import frc.robot.commands.SetCoralWristCommand;
import frc.robot.commands.SetElevatorCommand;
import frc.robot.subsystem.algae.GrabSubsystem;
import frc.robot.subsystem.coral.CoralSubsystem;
import frc.robot.subsystem.elevator.ElevatorSubsystem;

public class StateManager {
    private static StateManager instance = null;
    private RobotState currentRobotState = RobotState.RESET;
    private TargetState currentTargetState;


    protected CoralSubsystem coralSubsystem;
    protected GrabSubsystem grabSubsystem;
    protected ElevatorSubsystem elevatorSubsystem;

    private StateManager(CoralSubsystem coralSubsystem, GrabSubsystem grabSubsystem, ElevatorSubsystem elevatorSubsystem) {
        this.coralSubsystem = coralSubsystem;
        this.grabSubsystem = grabSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.currentTargetState = TargetState.RESET;
    }

    @SuppressWarnings("check)")
    public static StateManager getInstance(CoralSubsystem coralSubsystem, GrabSubsystem grabSubsystem, ElevatorSubsystem elevatorSubsystem) {
        if (instance == null) {
            instance = new StateManager(coralSubsystem, grabSubsystem, elevatorSubsystem);
        }
        return instance;
    }
    public static StateManager getInstance() {
        if (instance == null) {
            throw new IllegalStateException("StateManager has not been initialized. Call getInstance(CoralSubsystem, GrabSubsystem, ElevatorSubsystem) first.");
        }
        return instance;
    }
    
    
    public void setRobotState(RobotState state) { 
        System.out.println("[StateManager] setRobotState called! Changing state from " + this.currentRobotState + " to " + state);
        this.currentRobotState = state;
        //elevatorSubsystem.setRobotState(state);
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


    // TODO : Make More Clearly.
    public Command ElevatorSequence(TargetState targetState,RobotState robotState) {
        
        
        switch (targetState) {
            
            case PREP_L1:
                System.out.println("PREP_L1 button pressed! Current state: " + getRobotState());
                if (robotState == RobotState.PREP_L1) { 
                    // 第二次按下時，執行 SCORE_L1
                    setRobotState(RobotState.SCORE_L1);
                    //elevatorSubsystem.setCurrentState(RobotState.SCORE_L1);
                    return Commands.sequence(
                        //Commands.print(robotState.toString()),
                        Commands.print("score L1"),
                        coralSubsystem.outputCoralWithoutVision(),
                        Commands.runOnce(()->{
                            setRobotState(RobotState.SCORE_L1);
                            System.out.println("Robot State updated to: " + getRobotState());
                        }),
                        Commands.print("Scored L1")
                    );
                } else {
                    // 第一次按下時，進入 PREP_L1
                    setRobotState(RobotState.PREP_L1);
                    //elevatorSubsystem.setCurrentState(RobotState.PREP_L1);
                    return Commands.sequence(
                        Commands.parallel(
                            new SetCoralWristCommand(ScoreState.L3, coralSubsystem),
                            new SetElevatorCommand(ScoreState.L1, elevatorSubsystem)
                        ),
                        Commands.runOnce(()->{
                            setRobotState(RobotState.PREP_L1);
                            System.out.println("Robot State updated to: " + getRobotState());
                        }),
                        Commands.print(getRobotState().toString())
                        //Commands.print(elevatorSubsystem.getRobotState().toString())
                    );
                }
                
            case PREP_L2:
                if(this.currentRobotState == RobotState.PREP_L2) {
                    return Commands.sequence(
                        Commands.print("score L2"),
                        Commands.runOnce(()->coralSubsystem.outputCoralWithoutVision()),
                        Commands.waitSeconds(0.2),
                        //Commands.runOnce(()-> elevatorSubsystem.setRobotState(RobotState.SCORE_L2)),
                        Commands.print("Scored L2")
                    );
                }
                else {
                    return Commands.sequence(
                        Commands.parallel(
                            new SetCoralWristCommand(ScoreState.L3, coralSubsystem),
                            new SetElevatorCommand(ScoreState.L2, elevatorSubsystem)
                        ),
                        //Commands.runOnce(()-> elevatorSubsystem.setRobotState(RobotState.PREP_L2)),
                        Commands.print("PREP L2")
                    );
                }

            case PREP_L3:
                if(this.currentRobotState == RobotState.PREP_L3) {
                    return Commands.sequence(
                        Commands.print("score L3"),
                        Commands.runOnce(()->coralSubsystem.outputCoralWithoutVision()),
                        Commands.waitSeconds(0.2),
                        //Commands.runOnce(()-> elevatorSubsystem.setRobotState(RobotState.SCORE_L3)),
                        Commands.print("Scored L3")
                    );
                }
                else {
                    return Commands.sequence(
                        Commands.parallel(
                            new SetCoralWristCommand(ScoreState.L3, coralSubsystem),
                            new SetElevatorCommand(ScoreState.L3, elevatorSubsystem)
                        ),
                        //Commands.runOnce(()-> elevatorSubsystem.setRobotState(RobotState.PREP_L3)),
                        Commands.print("PREP L3")
                    );
                }
            case PREP_L4:
                if(this.currentRobotState == RobotState.PREP_L4) {
                    return Commands.sequence(
                        Commands.print("score L4"),
                        Commands.runOnce(()->coralSubsystem.outputCoralWithoutVision()),
                        Commands.waitSeconds(0.2),
                        //Commands.runOnce(()-> elevatorSubsystem.setRobotState(RobotState.SCORE_L4)),
                        Commands.print("Scored L4")
                    );
                }
                else {
                    return Commands.sequence(
                        Commands.parallel(
                            new SetCoralWristCommand(ScoreState.L3, coralSubsystem),
                            new SetElevatorCommand(ScoreState.L4, elevatorSubsystem)
                        ),
                        //Commands.runOnce(()-> elevatorSubsystem.setRobotState(RobotState.PREP_L4)),
                        Commands.print("PREP L4")
                    );
                }
            
            // TODO: Should change by added vision.
            case PREP_STATION:
                if(this.currentRobotState == RobotState.PREP_STATION) {
                    return Commands.sequence(
                        Commands.print("score STATION"),
                        Commands.runOnce(()->coralSubsystem.collectCoralWithoutVision()),
                        Commands.waitSeconds(0.2),
                        //Commands.runOnce(()->elevatorSubsystem.setRobotState(RobotState.SCORE_STATION)),
                        Commands.print("Scored STATION")
                    );
                }
                else {
                    return Commands.sequence(
                        Commands.parallel(
                            new SetCoralWristCommand(ScoreState.STATION, coralSubsystem),
                            new SetElevatorCommand(ScoreState.STATION, elevatorSubsystem)
                        ),
                        Commands.runOnce(()-> setRobotState(RobotState.PREP_STATION)),
                        Commands.print("PREP STATION")
                    );
                }
            case ALGAE_L2:
                
                return Commands.sequence(
                    Commands.parallel(
                        new SetCoralWristCommand(ScoreState.ALGAE_L2, coralSubsystem),
                        new SetElevatorCommand(ScoreState.ALGAE_L2, elevatorSubsystem)
                    )
                );
            case ALGAE_L3:
                return Commands.sequence(
                    Commands.parallel(
                        new SetCoralWristCommand(ScoreState.ALGAE_L3, coralSubsystem),
                        new SetElevatorCommand(ScoreState.ALGAE_L3, elevatorSubsystem)
                    )
                );
            case NORMAL:
                return Commands.sequence(
                    Commands.parallel(
                        new SetCoralWristCommand(ScoreState.NORMAL, coralSubsystem),
                        new SetElevatorCommand(ScoreState.NORMAL, elevatorSubsystem)
                    ),
                    Commands.runOnce(()-> setRobotState(RobotState.NORMAL)),
                    Commands.print("NORMAL")
                );
                
            default:
                return Commands.print("Invalid Target State");
              
            
        }
        
        
    }

    

    







    
    
}
