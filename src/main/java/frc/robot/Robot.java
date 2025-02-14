// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
;


public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  private boolean autoZeroed = false;
  private boolean haveAutoRun = false;

  public Robot() {
    m_robotContainer = new RobotContainer();
    DataLogManager.start();
    DriverStation.silenceJoystickConnectionWarning(true);

  }
  

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putBoolean("Zeroed", autoZeroed);
    SmartDashboard.putBoolean("haveAutoRun", haveAutoRun);
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    autoZeroed = m_robotContainer.isZeroed();

    if (autoZeroed && m_autonomousCommand != null) {
      Commands.deferredProxy( ()-> m_autonomousCommand).schedule();
    }
    else if(m_autonomousCommand != null) {
      m_robotContainer.zeroCommand().andThen(Commands.deferredProxy(()-> m_autonomousCommand));
    }
    else {
      m_robotContainer.zeroCommand().schedule();
    }
    haveAutoRun = true;
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    autoZeroed = m_robotContainer.isZeroed();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    if(!haveAutoRun) {
      m_robotContainer.zeroCommand().schedule();
    }
    
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
