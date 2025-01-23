package frc.robot.subsystem.coral;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.lib.util.CANDeviceId;

public class CoralSubsystem {
    
    private final SparkFlex m_intake = new SparkFlex(m_outTakeId.getDeviceNumber(),MotorType.kBrushless);
    private final SparkFlex m_armFlex = new SparkFlex(m_outTakeId.getDeviceNumber(),MotorType.kBrushless);
    private final SparkFlex m_wristFlex = new SparkFlex(m_outTakeId.getDeviceNumber(),MotorType.kBrushless);    


}
