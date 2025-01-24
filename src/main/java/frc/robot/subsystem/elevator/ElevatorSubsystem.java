package frc.robot.subsystem.elevator;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ElevatorConstants.*;
import static frc.robot.RobotMap.*;

public class ElevatorSubsystem extends SubsystemBase {
    private final TalonFX m_elevatorMotorLeft = new TalonFX(ELEVATOR_ID_LEFT.getDeviceNumber());
    private final TalonFX m_elevatorMotorRight = new TalonFX(ELEVATOR_ID_RIGHT.getDeviceNumber());
    private final DigitalInput m_elevatorLimitSwitch = new DigitalInput(ELEVATOR_LIMITSWITCH_ID);



    public ElevatorSubsystem() {
        m_elevatorMotorLeft.getConfigurator().apply(ELEVATOR_CONFIG_L);
        m_elevatorMotorRight.getConfigurator().apply(ELEVATOR_CONFIG_R);

        m_elevatorMotorLeft.setPosition(0);
        m_elevatorMotorRight.setPosition(0);
    }

    private boolean getLimit() {
        return m_elevatorLimitSwitch.get();
    }
    private void resetToZero() {
        m_elevatorMotorLeft.setPosition(0);
        m_elevatorMotorRight.setPosition(0);
    }

    private void setElevatorTargetPosition(double position) {
        m_elevatorMotorLeft.setPosition(position);
        m_elevatorMotorRight.setPosition(position);
    }
    private void toElevatorPosition(double position) {

    }
    private double getElevatorPosition() {
        return m_elevatorMotorLeft.getPosition().getValueAsDouble() * ELEVATOR_LENGTH;
    }
    private double getElevatorRotations(){
        return m_elevatorMotorLeft.getPosition().getValueAsDouble();
    }







}
