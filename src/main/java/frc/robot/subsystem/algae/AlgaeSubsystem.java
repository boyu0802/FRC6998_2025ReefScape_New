package frc.robot.subsystem.algae;

import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.CoralConstants.CORAL_INTAKECONFIG;
import static frc.robot.Constants.CoralConstants.CORAL_WRISTCONFIG;
import static frc.robot.RobotMap.*;

@Logged(name = "CoralSubsystem")
public class AlgaeSubsystem extends SubsystemBase {

    @Logged(name = "Algae_intake", importance = Logged.Importance.DEBUG)
    private final SparkFlex m_algaeIntake = new SparkFlex(ALGAE_INTAKE_ID.getDeviceNumber(),MotorType.kBrushless);

    @Logged(name = "Algae_wrist", importance = Logged.Importance.DEBUG)
    private final TalonFX m_algaeWrist = new TalonFX(ALGAE_WRIST_ID.getDeviceNumber());

    private final PositionTorqueCurrentFOC m_positionTorque = new PositionTorqueCurrentFOC(0).withSlot(0);
    private final PositionVoltage m_positionVoltage = new PositionVoltage(0).withSlot(0);
    private final VoltageOut m_voltageOut = new VoltageOut(0); // for SysId test.

    //TODO: SysId Testing.
    private final MutVoltage coralIntakeSysIdVoltage = Volts.mutable(0);
    private final MutAngle coralIntakeSysIdAngle = Units.Radians.mutable(0);
    private final MutAngularVelocity coralIntakeSysIdVelocity = RotationsPerSecond.mutable(0);
    private final MutVoltage coralWristSysIdVoltage = Volts.mutable(0);
    private final MutAngle coralWristSysIdAngle = Degrees.mutable(0);
    private final MutAngularVelocity coralWristSysIdVelocity = DegreesPerSecond.mutable(0);



    public AlgaeSubsystem() {
        setName("AlgaeSubsystem");

        m_algaeIntake.configure(
                CORAL_INTAKECONFIG,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters);

        m_algaeWrist.getConfigurator().apply(CORAL_WRISTCONFIG);
    }

    private final SysIdRoutine m_IntakesysIdRoutine =
            new SysIdRoutine(
                    // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
                    new SysIdRoutine.Config(),
                    new SysIdRoutine.Mechanism(
                            // Tell SysId how to plumb the driving voltage to the motor(s).
                            m_algaeIntake::setVoltage,
                            // Tell SysId how to record a frame of data for each motor on the mechanism being
                            // characterized.
                            log -> {
                                // Record a frame for the shooter motor.
                                log.motor("intake-coral")
                                        .voltage(coralIntakeSysIdVoltage.mut_replace(
                                                m_algaeIntake.getAppliedOutput()*m_algaeIntake.getBusVoltage(), Volts))
                                        .angularPosition(coralIntakeSysIdAngle.mut_replace(
                                                m_algaeIntake.getEncoder().getPosition(), Rotations))
                                        .angularVelocity(coralIntakeSysIdVelocity.mut_replace(
                                                m_algaeIntake.getEncoder().getVelocity(), RotationsPerSecond));
                            },
                            // Tell SysId to make generated commands require this subsystem, suffix test state in
                            // WPILog with this subsystem's name ("shooter")
                            this));

    private final SysIdRoutine wristSysIdRoutine =
            new SysIdRoutine(
                    // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
                    new SysIdRoutine.Config(),
                    new SysIdRoutine.Mechanism(
                            volts -> m_algaeWrist.setControl(m_voltageOut.withOutput(volts)),
                            log ->{
                                // Record a frame for the shooter motor.
                                log.motor("wrist-coral")
                                        .voltage(coralWristSysIdVoltage.mut_replace(
                                                m_algaeWrist.getMotorVoltage().getValueAsDouble(), Volts))
                                        .angularPosition(coralWristSysIdAngle.mut_replace(
                                                m_algaeWrist.getPosition().getValueAsDouble(), Rotations))
                                        .angularVelocity(coralWristSysIdVelocity.mut_replace(
                                                m_algaeWrist.getVelocity().getValueAsDouble(), RotationsPerSecond));
                            },
                            this));





    //TODO: Wait for test.
    public Command sysid_intakeDynamic(SysIdRoutine.Direction direction) {
        return wristSysIdRoutine.dynamic(direction);
    }
    public Command sysid_wristDynamic(SysIdRoutine.Direction direction) {
        return wristSysIdRoutine.dynamic(direction);
    }
    public Command sysid_intakeQuasistatic(SysIdRoutine.Direction direction) {
        return m_IntakesysIdRoutine.quasistatic(direction);
    }
    public Command sysid_wristQuasistatic(SysIdRoutine.Direction direction) {
        return wristSysIdRoutine.quasistatic(direction);
    }

}
