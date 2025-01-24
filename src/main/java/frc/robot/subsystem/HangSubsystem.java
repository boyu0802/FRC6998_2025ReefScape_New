package frc.robot.subsystem;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkFlex;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.RobotMap.CATCH_HANG_ID;
import static frc.robot.RobotMap.HANG_ID;

public class HangSubsystem extends SubsystemBase {
    private final SparkFlex m_catchHang = new SparkFlex(CATCH_HANG_ID.getDeviceNumber(), SparkFlex.MotorType.kBrushless);
    private final TalonFX m_hangMotor = new TalonFX(HANG_ID.getDeviceNumber());

}
