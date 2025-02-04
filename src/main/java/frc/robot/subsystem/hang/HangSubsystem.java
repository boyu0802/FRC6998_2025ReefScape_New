package frc.robot.subsystem.hang;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ElevatorConstants.ScoreState;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.HangConstants.CATCH_HANG_CONFIG;
import static frc.robot.Constants.HangConstants.HANG_CONFIG;
import static frc.robot.Constants.HangConstants.HANG_ENCODER_CONFIG;
import static frc.robot.RobotMap.CATCH_HANG_ID;
import static frc.robot.RobotMap.HANG_ENCODER_ID;
import static frc.robot.RobotMap.HANG_ID;

public class HangSubsystem extends SubsystemBase {
    private final SparkFlex m_catchHang = new SparkFlex(CATCH_HANG_ID.getDeviceNumber(), SparkFlex.MotorType.kBrushless);
    private final TalonFX m_hangMotor = new TalonFX(HANG_ID.getDeviceNumber());
    private final CANcoder m_hangMotorEncoder = new CANcoder(HANG_ENCODER_ID.getDeviceNumber());

    private final MotionMagicVoltage m_MotionMagicVoltage = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(true);
    //TODO: SysId Testing.
    private final VoltageOut m_voltageOut = new VoltageOut(0); // for SysId test.
    private final MutVoltage hang_sysIdVoltage = Volts.mutable(0);
    private final MutAngle hang_sysIdAngle = Rotations.mutable(0);
    private final MutAngularVelocity hang_sysIdVelocity = RotationsPerSecond.mutable(0);

    private final VelocityVoltage m_velocityVoltage = new VelocityVoltage(0).withSlot(0).withEnableFOC(true);

    private final PositionTorqueCurrentFOC m_TorqueCurrentFOC = new PositionTorqueCurrentFOC(0).withSlot(0);


    public HangSubsystem() {
        setName("HangSubsystem");
        m_catchHang.configure(
                CATCH_HANG_CONFIG,
                SparkFlex.ResetMode.kResetSafeParameters,
                SparkFlex.PersistMode.kPersistParameters);

        m_hangMotor.getConfigurator().apply(HANG_CONFIG);
        m_hangMotorEncoder.getConfigurator().apply(HANG_ENCODER_CONFIG);

        m_hangMotor.setPosition(gethangAbsoultePosition());
        m_catchHang.getEncoder().setPosition(0);
}
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
        
    public Command sysid_wristDynamic(SysIdRoutine.Direction direction){
                return wristSysIdRoutine.dynamic(direction);
    }

    
    public Command sysid_wristQuasistatic(SysIdRoutine.Direction direction){
                return wristSysIdRoutine.quasistatic(direction);
    }

    public double getHangIntakeVelocity(){
        return m_catchHang.getEncoder().getVelocity();
    }

    public double gethangAbsoultePosition(){
        return m_hangMotorEncoder.getAbsolutePosition().getValueAsDouble();
    }
    public double getHangWristPosition(){
        return m_hangMotor.getPosition().getValueAsDouble();
    }
    
    private void stopCatchIntake(){
        m_catchHang.set(0.0);
    }

    /** 
     * @param position (in degrees)
    */
    
    private void setHangPosition(double position){
        m_hangMotor.setControl(m_MotionMagicVoltage.withPosition(Units.degreesToRotations(position)));
    }
    private void setCatchHangVelocity(double velocity){
        m_catchHang.getClosedLoopController().setReference(velocity,SparkFlex.ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    }
    public void setHangVelocity(double velocity){
        m_hangMotor.setControl(m_TorqueCurrentFOC.withPosition(Units.degreesToRotations(120)).withVelocity(0.1));
    }
    

    
    public Command setHangto0deg(){
        return Commands.runOnce(()-> setHangPosition(0));}
    

    public Command setHangto90deg(){
        return Commands.runOnce(()-> setHangPosition(120));
    }
    public Command catchHang(){
        return Commands.sequence(
                Commands.runOnce(()-> setCatchHangVelocity(5.0)),
                Commands.waitSeconds(0.5),
                Commands.runOnce(()-> stopCatchIntake())
        );
    }

    
    @Override
    public void periodic(){
        SmartDashboard.putNumber("Hang/ catch hang velocity", getHangIntakeVelocity());
        SmartDashboard.putNumber("Hang/ hang position", getHangWristPosition());
        
        SmartDashboard.putNumber("Hang/ hang absolute Position.", gethangAbsoultePosition());
        SmartDashboard.putNumber("Hang/ Accel", m_hangMotor.getAcceleration().getValueAsDouble());
        SmartDashboard.putNumber("Hang/ hang current",m_hangMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Hang/ hang torque current",m_hangMotor.getTorqueCurrent().getValueAsDouble());

        SmartDashboard.putNumber("Hang/ hang velocity", m_hangMotor.getVelocity().getValueAsDouble());
    }

                
        
        
                            
}
