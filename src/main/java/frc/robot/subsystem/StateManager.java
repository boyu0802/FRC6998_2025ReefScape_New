package frc.robot.subsystem;

import frc.robot.Constants.RobotState;
import frc.robot.Constants.ScoreState;

import com.ctre.phoenix6.signals.RobotEnableValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.TargetState;
import frc.robot.commands.drive.SetRumbleCommand;
import frc.robot.commands.setcommand.CoralIntakeCommand;
import frc.robot.commands.setcommand.SetCoralWristCommand;
import frc.robot.commands.setcommand.SetElevatorCommand;
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
    
    
    private void setRobotState(RobotState state) { 
        System.out.println("[StateManager] setRobotState called! Changing state from " + this.currentRobotState + " to " + state);
        this.currentRobotState = state;
        //elevatorSubsystem.setRobotState(state);
    }

    private void setTargetState(TargetState state) {
        currentTargetState = state;
    }

    public RobotState getRobotState() {
        return currentRobotState;
    }

    public TargetState getTargetState() {
        return currentTargetState;
    }

    private Command toNormalCommand(){
        return Commands.sequence(
            Commands.runOnce(()->setRobotState(RobotState.NORMAL)),
            new SetCoralWristCommand(ScoreState.NORMAL, coralSubsystem),
            Commands.waitSeconds(0.2),
            new SetElevatorCommand(ScoreState.NORMAL, elevatorSubsystem),
            Commands.print("toNormalPosition")

        );
    }


    // TODO : Make More Clearly & add rumble.
    public Command SetReefState(TargetState targetState) {
        
        
        
        switch (targetState) {
            
            case PREP_L1:
                if(this.currentRobotState == RobotState.PREP_L1) {
                    return Commands.sequence(
                        Commands.runOnce(()->setRobotState(RobotState.SCORE_L1)),
                        Commands.print("score L1"),
                        coralSubsystem.outputCoralWithoutVision(),
                        Commands.print("Scored L1"),
                        Commands.waitSeconds(0.2),
                        toNormalCommand()
                        //Commands.runOnce(()-> elevatorSubsystem.setRobotState(RobotState.SCORE_L2)),
                        //setRumbleCommand
                    );
                }
                else {
                    return Commands.sequence(
                        Commands.runOnce(()->setRobotState(RobotState.PREP_L1)),
                        Commands.parallel(
                            new SetCoralWristCommand(ScoreState.L1, coralSubsystem),
                            new SetElevatorCommand(ScoreState.L1, elevatorSubsystem)
                        ),
                        //Commands.runOnce(()-> elevatorSubsystem.setRobotState(RobotState.PREP_L2)),
                        Commands.print("PREP L1")
                    );
                }
                
            case PREP_L2:
                if(this.currentRobotState == RobotState.PREP_L2) {
                    return Commands.sequence(
                        Commands.runOnce(()->setRobotState(RobotState.SCORE_L2)),
                        Commands.print("score L2"),
                        coralSubsystem.outputCoralWithoutVision(),
                        Commands.waitSeconds(0.2),
                        Commands.print("Scored L2"),
                        Commands.waitSeconds(0.2),
                        toNormalCommand()
                    );
                }
                else {
                    return Commands.sequence(
                        Commands.runOnce(()->setRobotState(RobotState.PREP_L2)),
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
                        Commands.runOnce(()->setRobotState(RobotState.SCORE_L3)),
                        Commands.print("score L3"),
                        coralSubsystem.outputCoralWithoutVision(),
                        Commands.waitSeconds(0.2),
                        Commands.print("Scored L3"),
                        Commands.waitSeconds(0.2),
                        toNormalCommand()
                    );
                }
                else {
                    return Commands.sequence(
                        Commands.runOnce(()->setRobotState(RobotState.PREP_L3)),
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
                        Commands.runOnce(()->setRobotState(RobotState.SCORE_L4)),
                        Commands.print("score L4"),
                        coralSubsystem.outputCoralWithoutVision(),
                        Commands.waitSeconds(0.2),
                        Commands.print("Scored L4"),
                        Commands.waitSeconds(0.2),
                        toNormalCommand()
                    );
                }
                else {
                    return Commands.sequence(
                        Commands.runOnce(()->setRobotState(RobotState.PREP_L4)),
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
                        Commands.runOnce(()->setRobotState(RobotState.SCORE_STATION)),
                        Commands.print("score STATION"),
                        coralSubsystem.collectCoralWithoutVision(),
                        Commands.waitSeconds(0.2),
                        //Commands.runOnce(()->elevatorSubsystem.setRobotState(RobotState.SCORE_STATION)),
                        Commands.print("Scored STATION")
                    );
                }
                else {
                    return Commands.sequence(
                        Commands.runOnce(()->setRobotState(RobotState.PREP_STATION)),
                        Commands.parallel(
                            new SetCoralWristCommand(ScoreState.STATION, coralSubsystem),
                            new SetElevatorCommand(ScoreState.STATION, elevatorSubsystem)
                        ),
                        Commands.runOnce(()-> setRobotState(RobotState.PREP_STATION)),
                        Commands.print("PREP STATION")
                    );
                }
            case PREP_ALGAE_L2:

                if(this.currentRobotState == RobotState.PREP_L2_ALGAE) {
                    return Commands.sequence(
                        Commands.runOnce(()->setRobotState(RobotState.EJECT_L2_ALGAE)),
                        Commands.print("Eject L2 Algae"),
                        coralSubsystem.collectAlgaeWithoutVision(),
                        Commands.waitSeconds(0.2),
                        Commands.print("Ejected L2 Algae")
                    );
                }
                else {
                    return Commands.sequence(
                        Commands.runOnce(()->setRobotState(RobotState.PREP_L2_ALGAE)),
                        Commands.parallel(
                            new SetCoralWristCommand(ScoreState.ALGAE_L2, coralSubsystem),
                            new SetElevatorCommand(ScoreState.ALGAE_L2, elevatorSubsystem)
                        )
                    );
                }
            case PREP_ALGAE_L3:
                if(this.currentRobotState == RobotState.PREP_L3_ALGAE) {
                    return Commands.sequence(
                        Commands.runOnce(()->setRobotState(RobotState.EJECT_L3_ALGAE)),
                        Commands.print("Eject L3 Algae"),
                        coralSubsystem.collectAlgaeWithoutVision(),
                        Commands.waitSeconds(0.2),
                        Commands.print("Ejected L3 Algae")
                    );
                }
                else {
                    return Commands.sequence(
                        Commands.runOnce(()->setRobotState(RobotState.PREP_L3_ALGAE)),
                        Commands.parallel(
                            new SetCoralWristCommand(ScoreState.ALGAE_L3, coralSubsystem),
                            new SetElevatorCommand(ScoreState.ALGAE_L3, elevatorSubsystem)
                        )
                    );
                }
            case NORMAL:
                return Commands.sequence(
                    Commands.parallel(
                        new SetCoralWristCommand(ScoreState.NORMAL, coralSubsystem),
                        new SetElevatorCommand(ScoreState.NORMAL, elevatorSubsystem)
                    ),
                    Commands.runOnce(()-> setRobotState(RobotState.NORMAL)),
                    Commands.print("NORMAL")
                );
            case NET:
                return Commands.sequence(
                    Commands.parallel(
                        new SetCoralWristCommand(ScoreState.NET, coralSubsystem),
                        new SetElevatorCommand(ScoreState.NET, elevatorSubsystem)
                    ),
                    Commands.runOnce(()-> setRobotState(RobotState.NET)),
                    Commands.print("NET")
                );

            default:
                return Commands.print("Invalid Target State");
              
        }
        
        
    }

    

    







    
    
}
