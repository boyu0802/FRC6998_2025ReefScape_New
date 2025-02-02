package frc.robot.subsystem.algae;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.CoralConstants.*;
import static frc.robot.Constants.HangConstants.GrabSubsystem.GRAB_INTAKE_CONFIG;
import static frc.robot.Constants.HangConstants.GrabSubsystem.GRAB_WRIST_CONFIG;
import static frc.robot.RobotMap.GRAB_INTAKE_ID;

public class GrabSubsystem extends SubsystemBase {
    private final SparkFlex m_grabIntake = new SparkFlex(GRAB_INTAKE_ID.getDeviceNumber(), SparkLowLevel.MotorType.kBrushless);
    private final SparkFlex m_grabWrist = new SparkFlex(GRAB_INTAKE_ID.getDeviceNumber(), SparkLowLevel.MotorType.kBrushless);

    public GrabSubsystem() {
        setName("GrabSubsystem");
        m_grabIntake.configure(
                GRAB_INTAKE_CONFIG,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters);
        m_grabWrist.configure(
                GRAB_WRIST_CONFIG,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters);

        m_grabWrist.getEncoder().setPosition(0);
        m_grabIntake.getEncoder().setPosition(0);

    }

    public void setGrabIntakeVelocity(double velocity){
        m_grabIntake.getClosedLoopController().
                setReference(velocity, SparkFlex.ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    }
    public void setGrabWristPosition(double position){
        m_grabWrist.getClosedLoopController().
                setReference(position, SparkFlex.ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    public double getGrabIntakeVelocity(){
        return m_grabIntake.getEncoder().getVelocity();
    }
    public double getGrabWristPosition(){
        return m_grabWrist.getEncoder().getPosition();
    }


    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        SmartDashboard.getNumber("Coral Intake Velocity", getGrabIntakeVelocity());
        SmartDashboard.getNumber("Coral Wrist Position", getGrabWristPosition());
        //SmartDashboard.getBoolean("Coral Intake Limit Switch",getCoralLimit());
    }


}
