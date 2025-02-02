package frc.robot.subsystem.coral;

import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.CoralConstants.*;
import static frc.robot.RobotMap.CORAL_INTAKE_ID;
import static frc.robot.RobotMap.CORAL_WRIST_ID;
import static frc.robot.RobotMap.CORAL_WRIST_ENCODER_ID;
import static frc.robot.RobotMap.CORAL_INTAKE_LIMITSWITCH_ID;

//@Logged(name = "CoralSubsystem")
public class CoralSubsystem extends SubsystemBase {

    //@Logged(name = "Coral_intake", importance = Logged.Importance.DEBUG)
    private final SparkFlex m_coralIntake = new SparkFlex(CORAL_INTAKE_ID.getDeviceNumber(),MotorType.kBrushless);

    //@Logged(name = "Coral_wrist", importance = Logged.Importance.DEBUG)
    private final TalonFX m_coralWrist = new TalonFX(CORAL_WRIST_ID.getDeviceNumber());

    
    private final CANcoder m_coralWristEncoder = new CANcoder(CORAL_WRIST_ENCODER_ID.getDeviceNumber());

    private final SparkLimitSwitch m_coralIntakeLimitSwitch = m_coralIntake.getForwardLimitSwitch();

    //private final TrapezoidProfile m_Profile = new TrapezoidProfile()
    //private final PositionTorqueCurrentFOC m_positionTorque = new PositionTorqueCurrentFOC(0).withSlot(0);
    private final PositionVoltage m_positionVoltage = new PositionVoltage(0).withSlot(0);
    private final VoltageOut m_voltageOut = new VoltageOut(0); // for SysId test.

    //TODO: SysId Testing.
    private final MutVoltage intakeSysIdVoltage = Volts.mutable(0);
    private final MutAngle intakeSysIdAngle = Rotations.mutable(0);
    private final MutAngularVelocity intakeSysIdVelocity = RotationsPerSecond.mutable(0);
    private final MutVoltage wrist_sysIdVoltage = Volts.mutable(0);
    private final MutAngle wrist_sysIdAngle = Degrees.mutable(0);
    private final MutAngularVelocity wrist_sysIdVelocity = DegreesPerSecond.mutable(0);



    public CoralSubsystem() {
        setName("CoralSubsystem");

        m_coralIntake.configure(
                CORAL_INTAKECONFIG,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters);

        m_coralWrist.getConfigurator().apply(CORAL_WRISTCONFIG);
        m_coralWristEncoder.getConfigurator().apply(CORAL_WRIST_ENCODER_CONFIG);
        //m_coralWristEncoder.setPosition(0);
        m_coralIntake.getEncoder().setPosition(0);
        m_coralWrist.setPosition(getCoralWristPosition());
    }

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
                                                m_coralWrist.getPosition().getValueAsDouble(), Degrees))
                                        .angularVelocity(wrist_sysIdVelocity.mut_replace(
                                                m_coralWrist.getVelocity().getValueAsDouble(),DegreesPerSecond));
                            },
                            this));




    public void setCoralIntakeVelocity(double velocity){
        m_coralIntake.getClosedLoopController().
                setReference(velocity, SparkFlex.ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    }
    public void setCoralWristPosition(double position){
        m_coralWrist.setControl(m_positionVoltage.withFeedForward(CORAL_WRIST_FEED_FORWARD.calculate(position,0)));
    }

    public double getCoralIntakeVelocity(){
        return m_coralIntake.getEncoder().getVelocity();
    }

    public double getCoralAbsoultePosition(){
        return m_coralWristEncoder.getAbsolutePosition().getValueAsDouble();
    }
    public double getCoralWristPosition(){
        return m_coralWrist.getPosition().getValueAsDouble();
    }
    public boolean getCoralLimit() {
        return m_coralIntakeLimitSwitch.isPressed();
    }
    public void stopCoralIntake(){
        m_coralIntake.set(0.0);
    }


    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        
        SmartDashboard.putNumber("Coral/Coral Intake Velocity",getCoralIntakeVelocity());
        SmartDashboard.putNumber("Coral/Coral Intake Position",getCoralIntakeVelocity()*60);
        SmartDashboard.putNumber("Coral/intake voltage", m_coralIntake.getAppliedOutput()* m_coralIntake.getBusVoltage());
        SmartDashboard.putNumber("Coral/Coral Intake Current",m_coralIntake.getOutputCurrent());
        SmartDashboard.putBoolean("Coral/limit switch ", getCoralLimit());
        //SmartDashboard.getNumber("Coral/Coral Wrist Position", getCoralWristPosition());
        //SmartDashboard.getBoolean("Coral/Intake Limit Switch", getCoralLimit());
    }

    //TODO: Wait for test.
    public Command sysid_intakeDynamic(SysIdRoutine.Direction direction){
        return m_IntakesysIdRoutine.dynamic(direction);
    }
    public Command sysid_wristDynamic(SysIdRoutine.Direction direction){
        return wristSysIdRoutine.dynamic(direction);
    }

    public Command sysid_intakeQuasistatic(SysIdRoutine.Direction direction){
        return m_IntakesysIdRoutine.quasistatic(direction);
    }
    public Command sysid_wristQuasistatic(SysIdRoutine.Direction direction){
        return wristSysIdRoutine.quasistatic(direction);
    }


    //TODO: Wait for code.
    public SequentialCommandGroup collectCoralWithoutVision() {
        return new SequentialCommandGroup(
            new InstantCommand(()-> setCoralIntakeVelocity(10)),
            new WaitUntilCommand(()-> getCoralLimit()),
            new InstantCommand(() -> stopCoralIntake())
        );
    }
    public Command outputCoralWithoutVision() {
        return Commands.sequence(
                Commands.print("running coral output"),
                Commands.runOnce(()->setCoralIntakeVelocity(-7.5)),
                Commands.waitSeconds(0.5),
                Commands.runOnce(()->stopCoralIntake()),
                Commands.print("coral outputted")
        );
    }

    public Command collectAlgaeWithoutVision() {
        return Commands.sequence(
                Commands.print("running algae intake"),
                Commands.runOnce(()->setCoralIntakeVelocity(30)),
                //Commands.waitUntil(this::getCoralLimit),
                Commands.waitSeconds(2.0),
                Commands.runOnce(()->setCoralIntakeVelocity(6.5)),
                Commands.print("algae collected")
        );
    }

    public Command outputAlgaeWithoutVision() {
        return Commands.sequence(
                Commands.print("running algae output"),
                Commands.runOnce(()->setCoralIntakeVelocity(30)),
                Commands.waitSeconds(0.1),
                Commands.runOnce(()->setCoralIntakeVelocity(-30)),
                Commands.waitSeconds(0.5),
                Commands.runOnce(()->stopCoralIntake()),
                Commands.print("algae outputted")
        );
    }







}
