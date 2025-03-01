package frc.robot.subsystem.algae;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.control.FeedForward;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.CoralConstants.*;
import static frc.robot.Constants.HangConstants.GrabConstants.GRAB_INTAKE_CONFIG;
import static frc.robot.Constants.HangConstants.GrabConstants.GRAB_INTAKE_VELOCITY;
import static frc.robot.Constants.HangConstants.GrabConstants.GRAB_REVERSE_VELOCITY;
import static frc.robot.Constants.HangConstants.GrabConstants.GRAB_WRIST_CONFIG;
import static frc.robot.Constants.HangConstants.GrabConstants.GRAB_WRIST_FEED_FORWARD;
import static frc.robot.Constants.HangConstants.GrabConstants.GRAB_WRIST_OFFSET;
import static frc.robot.Constants.HangConstants.GrabConstants.GRAB_WRIST_OFFSET_TOZERO;
import static frc.robot.RobotMap.GRAB_INTAKE_ID;
import static frc.robot.RobotMap.GRAB_WRIST_ID;

public class GrabSubsystem extends SubsystemBase {
    private final SparkFlex m_grabIntake = new SparkFlex(GRAB_INTAKE_ID.getDeviceNumber(), SparkLowLevel.MotorType.kBrushless);
    private final SparkFlex m_grabWrist = new SparkFlex(GRAB_WRIST_ID.getDeviceNumber(), SparkLowLevel.MotorType.kBrushless);

    //private final SparkLimitSwitch m_grabLimit = m_grabWrist.getReverseLimitSwitch();

    private final MutVoltage wrist_sysIdVoltage = Volts.mutable(0);
    private final MutAngle wrist_sysIdAngle = Degrees.mutable(0);
    private final MutAngularVelocity wrist_sysIdVelocity = DegreesPerSecond.mutable(0);

    public GrabSubsystem() {
        setName("GrabSubsystem");
        m_grabIntake.configure(
                GRAB_INTAKE_CONFIG,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters
                );
        m_grabWrist.configure(
                GRAB_WRIST_CONFIG,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters);

        //m_grabWrist.getEncoder().setPosition(m_grabWrist.);
        m_grabIntake.getEncoder().setPosition(0);

    }

    private final SysIdRoutine wristSysIdRoutine =
            new SysIdRoutine(
                    // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
                    new SysIdRoutine.Config(
                        Volts.of(1).per(Seconds),
                            Volts.of(4),
                            Seconds.of(6),
                            null
                    ),
                    new SysIdRoutine.Mechanism(
                            // Tell SysId how to plumb the driving voltage to the motor(s).
                            m_grabWrist::setVoltage,
                            // Tell SysId how to record a frame of data for each motor on the mechanism being
                            // characterized.
                            log -> {
                                // Record a frame for the shooter motor.
                                log.motor("grab_wrist")
                                        .voltage(wrist_sysIdVoltage.mut_replace(
                                                m_grabWrist.getAppliedOutput()* m_grabWrist.getBusVoltage(), Volts))
                                        .angularPosition(wrist_sysIdAngle.mut_replace(
                                                getActualPosition(), Rotations))
                                        .angularVelocity(wrist_sysIdVelocity.mut_replace(
                                               getGrabWristVelocity(), RotationsPerSecond));
                            },
                            // Tell SysId to make generated commands require this subsystem, suffix test state in
                            // WPILog with this subsystem's name ("shooter")
                            this));

    public void setGrabIntakeVelocity(double velocity){
        m_grabIntake.getClosedLoopController().
                setReference(velocity, SparkFlex.ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    }
    public void setGrabWristPosition(double position){
        m_grabWrist.getClosedLoopController().
                setReference(setActualPosition(position), SparkFlex.ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0,GRAB_WRIST_FEED_FORWARD.calculate(getActualPosition(),0));
    }

    public double setActualPosition(double position){
        return Units.degreesToRotations(GRAB_WRIST_OFFSET_TOZERO-position);
    }

    public double getGrabIntakeVelocity(){
        return m_grabIntake.getEncoder().getVelocity();
    }
    public double getGrabWristPosition(){
        return m_grabWrist.getAbsoluteEncoder().getPosition();
    }

    public double getGrabWristVelocity(){
        return m_grabWrist.getAbsoluteEncoder().getVelocity();
    }

    public Command sysid_wristDynamic(SysIdRoutine.Direction direction){
        return wristSysIdRoutine.dynamic(direction);
    }
    public Command sysid_wristQuasistatic(SysIdRoutine.Direction direction){
        return wristSysIdRoutine.quasistatic(direction);
    }

    private double getActualPosition(){
        return GRAB_WRIST_OFFSET_TOZERO-getGrabWristPosition()*360.0;
    }
    

    public Command setGrabto10deg(){
        return Commands.parallel(
            runOnce(()-> setGrabWristPosition(-17.5)),
            collectWithoutVision()
            );}

    public Command setGrabto27deg(){
        return Commands.parallel(
            runOnce(()-> setGrabWristPosition(27.5)),
            collectWithoutVision()
            );}
    

    public Command setGrabto75deg(){
        return Commands.sequence(
            runOnce(()-> setGrabWristPosition(55.0)),
            runOnce(()-> setGrabIntakeVelocity(-20.0))
            );
    }

    public Command collectWithoutVision() {
        return Commands.sequence(
                Commands.print("running algae intake"),
                Commands.runOnce(()->setGrabIntakeVelocity(GRAB_INTAKE_VELOCITY)),
                //Commands.waitUntil(this::getCoralLimit),
                Commands.waitSeconds(2.0),
                Commands.runOnce(()->setGrabIntakeVelocity(-5)),
                Commands.print("algae collected")
        );
    }
    public Command reverseWithoutVision() {
        return Commands.sequence(
                Commands.print("running algae intake"),
                Commands.runOnce(()->setGrabIntakeVelocity(GRAB_REVERSE_VELOCITY)),
                //Commands.waitUntil(this::getCoralLimit),
                Commands.waitSeconds(2.0),
                Commands.runOnce(()->setGrabIntakeVelocity(0)),
                Commands.print("algae collected")
        );
    }
    

    


    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        SmartDashboard.putNumber("Grab/ Intake Velocity", getGrabIntakeVelocity());
        SmartDashboard.putNumber("Grab/ Wrist Position", getGrabWristPosition());
        SmartDashboard.putNumber("Grab/ Wrist Velocity", getGrabWristVelocity());
        SmartDashboard.putNumber("Grab/ actual Postion", getActualPosition());
        //SmartDashboard.getBoolean("Coral Intake Limit Switch",getCoralLimit());
    }


}
