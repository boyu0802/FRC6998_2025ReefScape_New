package frc.robot.subsystem;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkFlex;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Constants.HangConstants.CATCH_HANG_CONFIG;
import static frc.robot.Constants.HangConstants.HANG_CONFIG;
import static frc.robot.RobotMap.CATCH_HANG_ID;
import static frc.robot.RobotMap.HANG_ID;

public class HangSubsystem extends SubsystemBase {
    private final SparkFlex m_catchHang = new SparkFlex(CATCH_HANG_ID.getDeviceNumber(), SparkFlex.MotorType.kBrushless);
    private final TalonFX m_hangMotor = new TalonFX(HANG_ID.getDeviceNumber());

    //TODO: SysId Testing.
    private final VoltageOut m_voltageOut = new VoltageOut(0); // for SysId test.
    private final MutVoltage catchSysIdVoltage = Volts.mutable(0);
    private final MutAngle catchSysIdAngle = Degrees.mutable(0);
    private final MutAngularVelocity catchSysIdVelocity = DegreesPerSecond.mutable(0);
    private final MutVoltage hang_sysIdVoltage = Volts.mutable(0);
    private final MutAngle hang_sysIdAngle = Rotations.mutable(0);
    private final MutAngularVelocity hang_sysIdVelocity = RotationsPerSecond.mutable(0);

    public HangSubsystem() {
        setName("HangSubsystem");
        m_catchHang.configure(
                CATCH_HANG_CONFIG,
                SparkFlex.ResetMode.kResetSafeParameters,
                SparkFlex.PersistMode.kPersistParameters);

        m_hangMotor.getConfigurator().apply(HANG_CONFIG);

        m_hangMotor.setPosition(0);
        m_catchHang.getEncoder().setPosition(0);

    }
    private final SysIdRoutine m_catchHangsysIdRoutine =
            new SysIdRoutine(
                    // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
                    new SysIdRoutine.Config(),
                    new SysIdRoutine.Mechanism(
                            // Tell SysId how to plumb the driving voltage to the motor(s).
                            m_catchHang::setVoltage,
                            // Tell SysId how to record a frame of data for each motor on the mechanism being
                            // characterized.
                            log -> {
                                // Record a frame for the shooter motor.
                                log.motor("catch-hang")
                                        .voltage(catchSysIdVoltage.mut_replace(
                                                m_catchHang.getAppliedOutput()* m_catchHang.getBusVoltage(), Volts))
                                        .angularPosition(catchSysIdAngle.mut_replace(
                                                m_catchHang.getEncoder().getPosition(), Degrees))
                                        .angularVelocity(catchSysIdVelocity.mut_replace(
                                                m_catchHang.getEncoder().getVelocity(), DegreesPerSecond));
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
                            volts -> m_hangMotor.setControl(m_voltageOut.withOutput(volts)),
                            log ->{
                                // Record a frame for the shooter motor.
                                log.motor("wrist-coral")
                                        .voltage(hang_sysIdVoltage.mut_replace(
                                                m_hangMotor.getMotorVoltage().getValueAsDouble(), Volts))
                                        .angularPosition(hang_sysIdAngle.mut_replace(
                                                m_hangMotor.getPosition().getValueAsDouble(), Rotations))
                                        .angularVelocity(hang_sysIdVelocity.mut_replace(
                                                m_hangMotor.getVelocity().getValueAsDouble(), RotationsPerSecond));
                            },
                            this));
}
