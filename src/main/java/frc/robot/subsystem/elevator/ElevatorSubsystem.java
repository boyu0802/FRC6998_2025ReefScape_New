package frc.robot.subsystem.elevator;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
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
import frc.robot.Constants.ElevatorConstants.ScoreState;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.ElevatorConstants.*;
import static frc.robot.RobotMap.*;

public class ElevatorSubsystem extends SubsystemBase {
    private final TalonFX m_elevatorLeft = new TalonFX(ELEVATOR_ID_LEFT.getDeviceNumber());
    private final TalonFX m_elevatorRight = new TalonFX(ELEVATOR_ID_RIGHT.getDeviceNumber());
    private final DigitalInput m_elevatorLimitSwitch = new DigitalInput(ELEVATOR_LIMITSWITCH_ID);

    private final VoltageOut elevator_voltageOut = new VoltageOut(0);
   // private final PositionTorqueCurrentFOC elevator_positionTorque = new PositionTorqueCurrentFOC(0).withSlot(0);
    private final MutVoltage m_voltageOut = Volts.mutable(0);
    private final MutDistance m_distance = Meters.mutable(0);
    private final MutLinearVelocity m_velocity = MetersPerSecond.mutable(0);

    private final MotionMagicDutyCycle m_motionMagicDutyCycle = new MotionMagicDutyCycle(0);

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
        //m_elevatorLeft.setControl(m_motionMagicDutyCycle);
        //m_elevatorRight.setControl(new Follower(m_elevatorLeft.getDeviceID(),true));
        // right using left as master
        m_elevatorLeft.setPosition(0);
        m_elevatorRight.setPosition(0);
        //elevatorLigament2d.setLength(getElevatorPosition());

        SmartDashboard.putData("Elevator 1",elevatorMechanism2d);

    }

    private final SysIdRoutine elevatorSysIdRoutine =
            new SysIdRoutine(
                    // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
                    new SysIdRoutine.Config(
                            Volts.of(1).per(Seconds),
                            Volts.of(4),
                            Seconds.of(5),
                            null

                    ),
                    new SysIdRoutine.Mechanism(
                            volts -> m_elevatorLeft.setControl(elevator_voltageOut.withOutput(volts)),
                            log ->{
                                // Record a frame for the shooter motor.
                                log.motor("elevator")
                                        .voltage(m_voltageOut.mut_replace(
                                                m_elevatorLeft.getMotorVoltage().getValueAsDouble(), Volts))
                                        .linearPosition(m_distance.mut_replace(
                                                getElevatorPosition(), Meters))
                                        .linearVelocity(m_velocity.mut_replace(
                                                getElevatorVelocity(), MetersPerSecond));
                            },
                            this));
            

    private boolean getLimit() {
        return m_elevatorLimitSwitch.get();
    }
    private void resetToZero() {
        m_elevatorLeft.setPosition(0);
        m_elevatorRight.setPosition(0);
    }
    private void stopElevator(){
        m_elevatorLeft.setControl(elevator_voltageOut.withOutput(0));
    }

    
    public void setElevatorPosition(double position) {
        m_elevatorLeft.setControl(m_motionMagicDutyCycle.withPosition(elevatorPositiontoRotations(position)).withFeedForward(ELEVATOR_FEED_FORWARD.calculate(position)));
    }
    public void setElevatorPosition(ScoreState state) {
        setElevatorPosition(state.elevatorPosition);
    }
    public double elevatorPositiontoRotations(double position){
        return position/(Math.PI*2*ELEVATOR_LENGTH);
    }
    public double getElevatorPosition() {
        return m_elevatorLeft.getPosition().getValueAsDouble() *Math.PI*2 * ELEVATOR_LENGTH;
    }

    public double getElevatorVelocity() {
        return m_elevatorLeft.getVelocity().getValueAsDouble() *Math.PI*2* ELEVATOR_LENGTH;
    }
    public double getElevatorRotations(){
        return m_elevatorLeft.getPosition().getValueAsDouble();
    }

    public Command setL1(){
        return Commands.parallel(
            Commands.run(()->setElevatorPosition(ScoreState.L1)),
            Commands.print("Setting Elevator to L1")
        );
    }
    public Command setL2(){
        return Commands.parallel(
            Commands.run(()->setElevatorPosition(ScoreState.L2)),
            Commands.print("Setting Elevator to L2")
        );
    }
    public Command setL3(){
        return Commands.parallel(
            Commands.run(()->setElevatorPosition(ScoreState.L3)),
            Commands.print("Setting Elevator to L3")
        );
    }
    public Command setL4(){
        return Commands.parallel(
            Commands.run(()->setElevatorPosition(ScoreState.L4)),
            Commands.print("Setting Elevator to L4")
        );
    }

    @Override
    public void simulationPeriodic() {
        elevatorSim.setInput(m_elevatorLeft.getSimState().getMotorVoltage());
        elevatorSim.update(0.020);
        //m_elevatorLeft.setControl(elevator_positionTorque);
        m_elevatorLeft.setPosition(elevatorSim.getPositionMeters());
        simVelocity = elevatorSim.getVelocityMetersPerSecond();

        elevatorLigament2d.setLength(getElevatorPosition());
        SmartDashboard.putData("Elevator 1",elevatorMechanism2d);
        SmartDashboard.putNumber("elevator velocity", simVelocity);
        SmartDashboard.putNumber("test", 1);
        
    }

    @Override
    public void periodic(){
        
        SmartDashboard.putNumber("Elevator/elevatorPosition", getElevatorPosition());
        SmartDashboard.putNumber("Elevator Velocity", getElevatorVelocity());
        elevatorLigament2d.setLength(getElevatorPosition());
        SmartDashboard.putData("Elevator 1",elevatorMechanism2d);
        
    }

    public void updateTelemetry(){
        elevatorLigament2d.setLength(getElevatorPosition());
        
    }

    







}
