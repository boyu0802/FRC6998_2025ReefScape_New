package frc.robot.commands;

import frc.robot.subsystem.coral.CoralSubsystem;
import frc.robot.subsystem.elevator.ElevatorSubsystem;

import java.lang.annotation.Target;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.RobotState;
import frc.robot.Constants.ScoreState;
import frc.robot.Constants.TargetState;
import frc.robot.commands.drive.SetRumbleCommand;

public class ReefStatePosition {
    private final CoralSubsystem coralSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final CommandXboxController xboxController;

    private RobotState currentRobotState = RobotState.RESET;
    
    public ReefStatePosition(CoralSubsystem coralSubsystem, ElevatorSubsystem elevatorSubsystem, CommandXboxController xboxController) {
        this.coralSubsystem = coralSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.xboxController = xboxController;
    }

    public void setRobotState(RobotState state) {
        //System.out.println("[ReefStatePosition] setRobotState called! Changing state from " + this.currentRobotState + " to " + state);
        currentRobotState = state;
    }

    public RobotState getRobotState() {
        return currentRobotState;
    }


    public SequentialCommandGroup setTargetState(TargetState currentTargetState) {
        SequentialCommandGroup newCommand;
        if(currentTargetState == TargetState.PREP_L1) {
            if(currentRobotState == RobotState.PREP_L1) {
                newCommand = new SequentialCommandGroup(
                        new InstantCommand(()->setRobotState(RobotState.SCORE_L1)),
                        new InstantCommand(coralSubsystem::outputCoralWithoutVision),
                        new WaitCommand(0.2),
                        new PrintCommand("Scored L1"),
                        new SetRumbleCommand(xboxController)
                );
                
            }
            else {
                newCommand = new SequentialCommandGroup(
                        new InstantCommand(()->setRobotState(RobotState.PREP_L1)),
                        new ParallelCommandGroup(
                            new SetCoralWristCommand(ScoreState.L3, coralSubsystem),
                            new SetElevatorCommand(ScoreState.L1, elevatorSubsystem) 
                        )
                );
            }
            
            
        }
        else {
            newCommand = new SequentialCommandGroup(
                        new InstantCommand(()->setRobotState(RobotState.PREP_L2)),
                        new ParallelCommandGroup(
                            new SetCoralWristCommand(ScoreState.L3, coralSubsystem),
                            new SetElevatorCommand(ScoreState.L2, elevatorSubsystem) 
                        )
                );
        }
        /* 
        switch(currentTargetState) {
            case PREP_L1 :
                if(currentRobotState == RobotState.PREP_L1) {
                    newCommand = new SequentialCommandGroup(
                        new InstantCommand(()->setRobotState(RobotState.SCORE_L1)),
                        new InstantCommand(()->coralSubsystem.outputCoralWithoutVision()),
                        new WaitCommand(0.2),
                        new PrintCommand("Scored L1"),
                        new InstantCommand(()-> xboxController.setRumble(RumbleType.kLeftRumble, 0.5))
                    );
                }
                else {
                    newCommand = new SequentialCommandGroup(
                        new InstantCommand(()->setRobotState(RobotState.PREP_L1)),
                        new ParallelCommandGroup(
                            new SetCoralWristCommand(ScoreState.L3, coralSubsystem),
                            new SetElevatorCommand(ScoreState.L1, elevatorSubsystem) 
                        )
                    );
                }
            case PREP_L2:
                if(currentRobotState == RobotState.PREP_L2) {
                    newCommand = new SequentialCommandGroup(
                        new InstantCommand(()->setRobotState(RobotState.SCORE_L2)),
                        new InstantCommand(()->coralSubsystem.outputCoralWithoutVision()),
                        new WaitCommand(0.2),
                        new PrintCommand("Scored L2"),
                        new InstantCommand(()-> xboxController.setRumble(RumbleType.kLeftRumble, 0.5))
                    );
                }
                else {
                    newCommand = new SequentialCommandGroup(
                        new InstantCommand(()->setRobotState(RobotState.PREP_L2)),
                        new ParallelCommandGroup(
                            new SetCoralWristCommand(ScoreState.L3, coralSubsystem),
                            new SetElevatorCommand(ScoreState.L2, elevatorSubsystem) 
                        )
                    );
                }
            case PREP_L3:
                if(currentRobotState == RobotState.PREP_L3) {
                    newCommand = new SequentialCommandGroup(
                        new InstantCommand(()->setRobotState(RobotState.SCORE_L3)),
                        new InstantCommand(()->coralSubsystem.outputCoralWithoutVision()),
                        new WaitCommand(0.2),
                        new PrintCommand("Scored L3"),
                        new InstantCommand(()-> xboxController.setRumble(RumbleType.kLeftRumble, 0.5))
                    );
                }
                else {
                    newCommand = new SequentialCommandGroup(
                        new InstantCommand(()->setRobotState(RobotState.PREP_L3)),
                        new ParallelCommandGroup(
                            new SetCoralWristCommand(ScoreState.L3, coralSubsystem),
                            new SetElevatorCommand(ScoreState.L3, elevatorSubsystem) 
                        )
                    );
                }
            case PREP_L4:
                if(currentRobotState == RobotState.PREP_L4) {
                    newCommand = new SequentialCommandGroup(
                        new InstantCommand(()->setRobotState(RobotState.SCORE_L4)),
                        new InstantCommand(()->coralSubsystem.outputCoralWithoutVision()),
                        new WaitCommand(0.2),
                        new PrintCommand("Scored L4"),
                        new InstantCommand(()-> xboxController.setRumble(RumbleType.kLeftRumble, 0.5))
                    );
                }
                else {
                    newCommand = new SequentialCommandGroup(
                        new InstantCommand(()->setRobotState(RobotState.PREP_L4)),
                        new ParallelCommandGroup(
                            new SetCoralWristCommand(ScoreState.L4, coralSubsystem),
                            new SetElevatorCommand(ScoreState.L4, elevatorSubsystem) 
                        )
                    );
                }
            case NORMAL:
                newCommand = new SequentialCommandGroup(
                    new InstantCommand(()->setRobotState(RobotState.NORMAL)),
                    new ParallelCommandGroup(
                        new SetCoralWristCommand(ScoreState.NORMAL, coralSubsystem),
                        new SetElevatorCommand(ScoreState.NORMAL, elevatorSubsystem) 
                    )
                );
            case PREP_STATION:
                newCommand = new SequentialCommandGroup(
                    new InstantCommand(()->setRobotState(RobotState.PREP_STATION)),
                    new ParallelCommandGroup(
                        new SetCoralWristCommand(ScoreState.STATION, coralSubsystem),
                        new SetElevatorCommand(ScoreState.STATION, elevatorSubsystem) 
                    )
                );
            case PREP_ALGAE_L2:
                newCommand = new SequentialCommandGroup(
                    new InstantCommand(()->setRobotState(RobotState.PREP_L2_ALGAE)),
                    new ParallelCommandGroup(
                        new SetCoralWristCommand(ScoreState.ALGAE_L2, coralSubsystem),
                        new SetElevatorCommand(ScoreState.ALGAE_L2, elevatorSubsystem) 
                    )
                );
            case PREP_ALGAE_L3:
                newCommand = new SequentialCommandGroup(
                    new InstantCommand(()->setRobotState(RobotState.PREP_L3_ALGAE)),
                    new ParallelCommandGroup(
                        new SetCoralWristCommand(ScoreState.ALGAE_L3, coralSubsystem),
                        new SetElevatorCommand(ScoreState.ALGAE_L3, elevatorSubsystem) 
                    )
                );
            
            

            default:
                newCommand = new SequentialCommandGroup(
                    new PrintCommand("Invalid Target State")
                );
        }
        */

        return newCommand;

    }
    
}
