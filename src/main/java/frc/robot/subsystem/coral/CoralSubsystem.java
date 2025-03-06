package frc.robot.subsystem.coral;

import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ScoreState;

import static frc.robot.Constants.ScoreState;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.CoralConstants.*;
import static frc.robot.RobotMap.CORAL_INTAKE_ID;
import static frc.robot.RobotMap.CORAL_WRIST_ID;
import static frc.robot.RobotMap.CORAL_WRIST_ENCODER_ID;
import static frc.robot.RobotMap.CORAL_INTAKE_LIMITSWITCH_ID;

import static java.lang.Math.abs;

//@Logged(name = "CoralSubsystem")
public class CoralSubsystem extends SubsystemBase {

    //@Logged(name = "Coral_intake", importance = Logged.Importance.DEBUG)
    private final TalonFX m_coralIntake = new TalonFX(CORAL_INTAKE_ID.getDeviceNumber());

    //@Logged(name = "Coral_wrist", importance = Logged.Importance.DEBUG)
    private final TalonFX m_coralWrist = new TalonFX(CORAL_WRIST_ID.getDeviceNumber());

    
    private final CANcoder m_coralWristEncoder = new CANcoder(CORAL_WRIST_ENCODER_ID.getDeviceNumber());

    private final DigitalOutput m_coralIntakeLimitSwitch = new DigitalOutput(CORAL_INTAKE_LIMITSWITCH_ID);

    //private final TrapezoidProfile m_Profile = new TrapezoidProfile()
    //private final PositionTorqueCurrentFOC m_positionTorque = new PositionTorqueCurrentFOC(0).withSlot(0);
    private final PositionVoltage m_positionVoltage = new PositionVoltage(0).withSlot(0);
    private final VoltageOut m_voltageOut = new VoltageOut(0); // for SysId test.
    private final MotionMagicVoltage m_MotionMagicVoltage = new MotionMagicVoltage(0).withSlot(0);
    private final VelocityVoltage m_VelocityVoltage = new VelocityVoltage(0).withSlot(0);
    //TODO: SysId Testing.
    private final MutVoltage intakeSysIdVoltage = Volts.mutable(0);
    private final MutAngle intakeSysIdAngle = Rotations.mutable(0);
    private final MutAngularVelocity intakeSysIdVelocity = RotationsPerSecond.mutable(0);
    private final MutVoltage wrist_sysIdVoltage = Volts.mutable(0);
    private final MutAngle wrist_sysIdAngle = Degrees.mutable(0);
    private final MutAngularVelocity wrist_sysIdVelocity = RotationsPerSecond.mutable(0);

    private double targetPosition = 90.0;



    public CoralSubsystem() {
        setName("CoralSubsystem");

        m_coralIntake.getConfigurator().apply(CORAL_INTAKE_CONFIG);
        m_coralWristEncoder.getConfigurator().apply(CORAL_WRIST_ENCODER_CONFIG);
        

        CORAL_WRISTCONFIG.Feedback.FeedbackRemoteSensorID = m_coralWristEncoder.getDeviceID();
        CORAL_WRISTCONFIG.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;

        m_coralWrist.getConfigurator().apply(CORAL_WRISTCONFIG);
        
        
        //m_coralWristEncoder.setPosition(0);
        m_coralIntake.setPosition(0);
        m_coralWrist.setPosition(getCoralAbsoultePosition());
        
    }
    /* 
    private final SysIdRoutine m_IntakesysIdRoutine =
            new SysIdRoutine(
                    // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
                    new SysIdRoutine.Config(
                        Volts.of(1).per(Seconds),
                            Volts.of(4),
                            Seconds.of(7),
                            null
                    ),
                    new SysIdRoutine.Mechanism(
                            // Tell SysId how to plumb the driving voltage to the motor(s).
                            m_coralIntake::setVoltage,
                            // Tell SysId how to record a frame of data for each motor on the mechanism being
                            // characterized.
                            log -> {
                                // Record a frame for the shooter motor.
                                log.motor("intake-coral")
                                        .voltage(intakeSysIdVoltage.mut_replace(
                                                m_coralIntake.getAppliedOutput()* m_coralIntake.getBusVoltage(), Volts))
                                        .angularPosition(intakeSysIdAngle.mut_replace(
                                                m_coralIntake.getEncoder().getPosition(), Rotations))
                                        .angularVelocity(intakeSysIdVelocity.mut_replace(
                                                m_coralIntake.getEncoder().getVelocity(), RotationsPerSecond));
                            },
                            // Tell SysId to make generated commands require this subsystem, suffix test state in
                            // WPILog with this subsystem's name ("shooter")
                            this));

    */

    private final SysIdRoutine wristSysIdRoutine =
            new SysIdRoutine(
                    // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
                    new SysIdRoutine.Config(
                            Volts.of(1).per(Seconds),
                            Volts.of(4),
                            Seconds.of(5),
                            null

                    ),
                    new SysIdRoutine.Mechanism(
                            volts -> m_coralWrist.setControl(m_voltageOut.withOutput(volts)),
                            log ->{
                                // Record a frame for the shooter motor.
                                log.motor("wrist-coral")
                                        .voltage(wrist_sysIdVoltage.mut_replace(
                                                m_coralWrist.getMotorVoltage().getValueAsDouble(), Volts))
                                        .angularPosition(wrist_sysIdAngle.mut_replace(
                                                m_coralWrist.getPosition().getValueAsDouble(), Rotations))
                                        .angularVelocity(wrist_sysIdVelocity.mut_replace(
                                                m_coralWrist.getVelocity().getValueAsDouble(),RotationsPerSecond));
                            },
                            this));




    public void setCoralIntakeVelocity(double velocity){
        m_coralIntake.setControl(m_VelocityVoltage.withVelocity(velocity));
    }
    public void setCoralWristPosition(double position){
        targetPosition = position;
        m_coralWrist.setControl(m_MotionMagicVoltage.withPosition(edu.wpi.first.math.util.Units.degreesToRotations(position)));
    }

    public void setCoralWristPosition(ScoreState state) {
        setCoralWristPosition(state.armPosition);
    }

    public double getCoralIntakeVelocity(){
        return m_coralIntake.getVelocity().getValueAsDouble();
    }

    public double getCoralAbsoultePosition(){
        return m_coralWristEncoder.getAbsolutePosition().getValueAsDouble();
    }
    public double getCoralWristPosition(){
        return m_coralWrist.getPosition().getValueAsDouble();
    }
    public boolean getCoralLimit() {
        return m_coralIntakeLimitSwitch.get();
    }
    public void stopCoralIntake(){
        m_coralIntake.set(0.0);
    }

    public boolean getCoralWristAtSetpoint(){
        return abs(edu.wpi.first.math.util.Units.rotationsToDegrees(m_coralWrist.getPosition().getValueAsDouble()) - targetPosition) < 2.5;
    }

    /* 
    public boolean getIntakeOpened(){
        return m_coralIntake.getEncoder().getVelocity() > 0.5 ||  m_coralIntake.getEncoder().getVelocity() < -0.5;
    }
        */


    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        
        SmartDashboard.putNumber("Coral/intake/ Coral Intake Velocity",getCoralIntakeVelocity());
        SmartDashboard.putNumber("Coral/intake/ Coral Intake Position",getCoralIntakeVelocity()*60);
        //SmartDashboard.putNumber("Coral/intake/ voltage", m_coralIntake.getAppliedOutput()* m_coralIntake.getBusVoltage());
        //SmartDashboard.putNumber("Coral/intake/ Current",m_coralIntake.getOutputCurrent());
        SmartDashboard.putBoolean("Coral/intake/ limit switch ", getCoralLimit());

        SmartDashboard.putNumber("Coral/wrist/ Position", getCoralWristPosition());
        SmartDashboard.putNumber("Coral/wrist/ Absolute Position", getCoralAbsoultePosition());
        //SmartDashboard.getNumber("Coral/Coral Wrist Position", getCoralWristPosition());
        //SmartDashboard.getBoolean("Coral/Intake Limit Switch", getCoralLimit());

        
    }

    //TODO: Wait for test.
    
    public Command sysid_wristDynamic(SysIdRoutine.Direction direction){
        return wristSysIdRoutine.dynamic(direction);
    }

    
    public Command sysid_wristQuasistatic(SysIdRoutine.Direction direction){
        return wristSysIdRoutine.quasistatic(direction);
    }


    // use in auto path only.
    public SequentialCommandGroup collectCoralWithoutVision() {
        return new SequentialCommandGroup(
            new InstantCommand(()-> setCoralIntakeVelocity(15)),
            new ParallelDeadlineGroup(
                new WaitCommand(2.0),
                new WaitUntilCommand(()->getCoralLimit())),
            new InstantCommand(() -> stopCoralIntake())
        );
    }
    
    public Command outputCoralWithoutVision() {
        return Commands.sequence(
                Commands.print("running coral output"),
                Commands.runOnce(()->setCoralIntakeVelocity(-50.0)),
                Commands.waitSeconds(0.5),
                Commands.runOnce(()->stopCoralIntake()),
                Commands.print("coral outputted")
        );
    }

    // use in auto path only.
    public Command collectAlgaeWithoutVision() {
        return Commands.sequence(
                Commands.print("running algae intake"),
                Commands.runOnce(()->setCoralIntakeVelocity(40)),
                //Commands.waitUntil(this::getCoralLimit),
                Commands.waitSeconds(2.0),
                Commands.runOnce(()->setCoralIntakeVelocity(6.5)),
                Commands.print("algae collected")
        );
    }

    public Command outputAlgaeWithoutVision() {
        return Commands.sequence(
                Commands.print("running algae output"),
                Commands.runOnce(()->setCoralIntakeVelocity(60)),
                Commands.waitSeconds(0.2),
                Commands.runOnce(()->setCoralIntakeVelocity(-60)),
                Commands.waitSeconds(0.6),
                Commands.runOnce(()->stopCoralIntake()),
                Commands.print("algae outputted")
        );
    }

    public Command wristToCoral(){
        return Commands.runOnce(()-> setCoralWristPosition(ScoreState.L3));
    }

    public Command wristToStation(){
        return Commands.runOnce(()-> setCoralWristPosition(ScoreState.STATION));
    }
    public Command wristToNormal(){
        return Commands.runOnce(()-> setCoralWristPosition(ScoreState.NORMAL));
    }






}
