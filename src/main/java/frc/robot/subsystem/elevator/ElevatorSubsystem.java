package frc.robot.subsystem.elevator;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.ElevatorConstants.*;
import static frc.robot.RobotMap.*;

public class ElevatorSubsystem extends SubsystemBase {
    private final TalonFX m_elevatorLeft = new TalonFX(ELEVATOR_ID_LEFT.getDeviceNumber());
    private final TalonFX m_elevatorRight = new TalonFX(ELEVATOR_ID_RIGHT.getDeviceNumber());
    private final DigitalInput m_elevatorLimitSwitch = new DigitalInput(ELEVATOR_LIMITSWITCH_ID);

    private final VoltageOut elevator_voltageOut = new VoltageOut(0);
    private final MutVoltage m_voltageOut = Volts.mutable(0);
    private final MutDistance m_distance = Meters.mutable(0);
    private final MutLinearVelocity m_velocity = MetersPerSecond.mutable(0);






    public ElevatorSubsystem() {
        m_elevatorLeft.getConfigurator().apply(ELEVATOR_CONFIG_L);
        m_elevatorRight.getConfigurator().apply(ELEVATOR_CONFIG_R);
        m_elevatorRight.setControl(new Follower(m_elevatorLeft.getDeviceID(),true));

        m_elevatorLeft.setPosition(0);
        m_elevatorRight.setPosition(0);
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
                                log.motor("wrist-coral")
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

    private void setElevatorTargetPosition(double position) {
        m_elevatorLeft.setPosition(position);
        m_elevatorRight.setPosition(position);
    }
    private void toElevatorPosition(double position) {

    }
    private double getElevatorPosition() {
        return m_elevatorLeft.getPosition().getValueAsDouble() *Math.PI*2 * ELEVATOR_LENGTH;
    }

    private double getElevatorVelocity() {
        return m_elevatorLeft.getVelocity().getValueAsDouble() *Math.PI*2* ELEVATOR_LENGTH;
    }
    private double getElevatorRotations(){
        return m_elevatorLeft.getPosition().getValueAsDouble();
    }







}
