package frc.robot.subsystem.elevator;


import com.ctre.phoenix6.controls.Follower;

import com.ctre.phoenix6.controls.MotionMagicVoltage;


import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.system.plant.DCMotor;

import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.RobotState;
import frc.robot.Constants.ScoreState;
import frc.robot.subsystem.StateManager;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.ElevatorConstants.*;
import static frc.robot.RobotMap.*;

import java.util.function.BooleanSupplier;

public class ElevatorSubsystem extends SubsystemBase {
    private final TalonFX m_elevatorLeft = new TalonFX(ELEVATOR_ID_LEFT.getDeviceNumber());
    private final TalonFX m_elevatorRight = new TalonFX(ELEVATOR_ID_RIGHT.getDeviceNumber());
    //private final DigitalInput m_elevatorLimitSwitch = new DigitalInput(ELEVATOR_LIMITSWITCH_ID);

    private final VoltageOut elevator_voltageOut = new VoltageOut(0);
   // private final PositionTorqueCurrentFOC elevator_positionTorque = new PositionTorqueCurrentFOC(0).withSlot(0);
    private final MutVoltage m_voltageOut = Volts.mutable(0);
    private final MutDistance m_distance = Meters.mutable(0);
    private final MutLinearVelocity m_velocity = MetersPerSecond.mutable(0);

    private final MotionMagicVoltage m_motionMagicVoltage = new MotionMagicVoltage(0.01);
    private final DigitalInput m_elevatorLimit = new DigitalInput(ELEVATOR_LIMITSWITCH_ID);

    public RobotState currentRobotState = RobotState.RESET;

    

    //private final PositionDutyCycle

    final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
    private double currentPosition = 0.01;


    private final ElevatorSim elevatorSim = new ElevatorSim(
        DCMotor.getKrakenX60Foc(ELEVATOR_ID_LEFT.getDeviceNumber()),
        9.0,
        12.0,
        ELEVATOR_LENGTH,
        0.0,
        2.1,
        true,
        0.0
    );

    private double simVelocity = 0.0;


    private final Mechanism2d elevatorMechanism2d = new Mechanism2d(5, 5);
    private MechanismRoot2d root = elevatorMechanism2d.getRoot("root", 2.5, 0.5);
    private MechanismLigament2d elevatorLigament2d = root.append(
        new MechanismLigament2d("elevatorControl",1.5, 0,7,new Color8Bit(Color.kOrange)));
        
    
    public ElevatorSubsystem() {
        m_elevatorLeft.getConfigurator().apply(ELEVATOR_CONFIG_L);
        m_elevatorRight.getConfigurator().apply(ELEVATOR_CONFIG_R);
        
        //m_elevatorLeft.setControl(elevator_positionTorque);
        
        // right using left as master
        m_elevatorLeft.setPosition(0.01);
        m_elevatorRight.setPosition(0.01);
        //elevatorLigament2d.setLength(getElevatorPosition());

        //m_elevatorLeft.setControl(m_motionMagicVoltage.withPosition(elevatorPositiontoRotations(currentPosition)).withSlot(0));
        //m_elevatorRight.setControl(new Follower(m_elevatorLeft.getDeviceID(),true));

        //SmartDashboard.putData("Elevator 1",elevatorMechanism2d);
        

    }

    private final SysIdRoutine m_elevatorSysIdRoutine =
            new SysIdRoutine(
                    // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
                    new SysIdRoutine.Config(
                            Volts.of(1).per(Seconds),
                            Volts.of(6),
                            Seconds.of(6),
                            null

                    ),
                    new SysIdRoutine.Mechanism(
                            volts -> {
                                m_elevatorLeft.setControl(elevator_voltageOut.withOutput(volts));
                                m_elevatorRight.setControl(new Follower(m_elevatorLeft.getDeviceID(), true));},
                            log ->{
                                // Record a frame for the shooter motor.
                                log.motor("elevator")
                                        .voltage(m_voltageOut.mut_replace(
                                                m_elevatorLeft.getMotorVoltage().getValueAsDouble(), Volts))
                                        .linearPosition(m_distance.mut_replace(
                                                getLeftElevatorPosition(), Meters))
                                        .linearVelocity(m_velocity.mut_replace(
                                                getLeftElevatorVelocity(), MetersPerSecond));
                            },
                            this));
            

    
    
    
    public void stopElevator(){
        m_elevatorLeft.setControl(elevator_voltageOut.withOutput(0));
        m_elevatorRight.setControl(elevator_voltageOut.withOutput(0));
    }
    private double getTargetPosition(){
        return currentPosition;
    }

    public void setSoftwareLimits(boolean forward , boolean reverse){
        ELEVATOR_CONFIG_L.SoftwareLimitSwitch.ForwardSoftLimitEnable = forward;
        ELEVATOR_CONFIG_L.SoftwareLimitSwitch.ReverseSoftLimitEnable = reverse;
        ELEVATOR_CONFIG_R.SoftwareLimitSwitch.ForwardSoftLimitEnable = forward;
        ELEVATOR_CONFIG_R.SoftwareLimitSwitch.ReverseSoftLimitEnable = reverse;

        m_elevatorLeft.getConfigurator().apply(ELEVATOR_CONFIG_L);
        m_elevatorRight.getConfigurator().apply(ELEVATOR_CONFIG_R);
    }
    
    public boolean getElevatorLimit(){
        return (!m_elevatorLimit.get());
    }
    public void setVoltage(double voltage){
        m_elevatorLeft.setControl(elevator_voltageOut.withOutput(voltage));
        m_elevatorRight.setControl(new Follower(m_elevatorLeft.getDeviceID(),true));
    }

    
    public boolean isAtSetpoint() {
    return (Math.abs(getLeftElevatorPosition() - getTargetPosition()) < ELEVATOR_DEADZONE_DISTANCE && Math.abs(getRightElevatorPosition() - getTargetPosition()) < ELEVATOR_DEADZONE_DISTANCE);
  }
    
    public void setElevatorPosition(double position) {
        m_elevatorLeft.setControl(m_motionMagicVoltage.withPosition(position));
        m_elevatorRight.setControl(m_motionMagicVoltage.withPosition(position));
        currentPosition = position;
    }
    public void setElevatorPosition(ScoreState state) {
        setElevatorPosition(state.elevatorPosition);
    }
    public void increaseElevatorPosition(double position) {
        setElevatorPosition(currentPosition + 0.05);
    }
    public void decreaseElevatorPosition(double position) {
        setElevatorPosition(currentPosition - 0.05);
    }
    public double elevatorPositiontoRotations(double position){
        return position/ELEVATOR_LENGTH;
    }

    public double getLeftElevatorPosition() {
        return m_elevatorLeft.getPosition().getValueAsDouble() ;
    }

    public double getLeftElevatorVelocity() {
        return m_elevatorLeft.getVelocity().getValueAsDouble();
    }
    public double getLeftElevatorRotations(){
        return m_elevatorLeft.getPosition().getValueAsDouble();
    }
    public double getRightElevatorPosition(){
        return m_elevatorRight.getPosition().getValueAsDouble();
    }

    public void resetPosition(){
        m_elevatorRight.setPosition(0);
        m_elevatorLeft.setPosition(0);
        
    }

    
    
    

    public Command sysid_elevatorDynamic(SysIdRoutine.Direction direction){
        return m_elevatorSysIdRoutine.dynamic(direction);
    }
    public Command sysid_elevatorQuasistatic(SysIdRoutine.Direction direction){
        return m_elevatorSysIdRoutine.quasistatic(direction);
    }
    public BooleanSupplier isAtSetpointSupplier() {
        return this::isAtSetpoint;
    }


    public Command increaseElevatorPositionCmd(){
        return Commands.parallel(
            Commands.runOnce(()->increaseElevatorPosition(0.05)),
            Commands.print("increaseElevatorPositionCmd")
        
            
        );
    }

    public Command decreaseElevatorPositionCmd(){
        return Commands.parallel(
            Commands.runOnce(()->decreaseElevatorPosition(0.05)),
            Commands.print("decreaseElevatorPositionCmd")
        
            
        );
    }

    

    
    @Override
    public void periodic(){
        
        SmartDashboard.putNumber("Elevator/elevatorPosition", getLeftElevatorPosition());
        SmartDashboard.putNumber("Elevator Velocity", getLeftElevatorVelocity());
        SmartDashboard.putNumber("Elevator/Voltage", m_elevatorLeft.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Elevator/elevatorRightPosition", getRightElevatorPosition());
        elevatorLigament2d.setLength(getLeftElevatorPosition());
        SmartDashboard.putData("Elevator 1",elevatorMechanism2d);
        SmartDashboard.putNumber("Elevator/ Rotations" ,m_elevatorLeft.getPosition().getValueAsDouble());
        //SmartDashboard.putNumber("Elevator/score state",elevatorPositiontoRotations(0.8));
        SmartDashboard.putNumber("Elevator/ Velocity", getLeftElevatorVelocity());
        SmartDashboard.putNumber("Elevator/ current position ", currentPosition);
        SmartDashboard.putBoolean("Elevator/ setpoint", isAtSetpoint());
        SmartDashboard.putBoolean("Elevator/ getLimit", getElevatorLimit());
        //SmartDashboard.putString("Command : RobotState", getRobotState().toString());
    }

    public void updateTelemetry(){
        elevatorLigament2d.setLength(getLeftElevatorPosition());
        
    }

    







}
