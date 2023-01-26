// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MotorControl;

public class AutoDriver extends CommandBase {
  /** Creates a new AutoDriver. */

  private MotorControl m_subsystem;

  private double encoderCount = 0;

  private boolean isFinished = false;
  private boolean ifForward;

  private float ft = 0;
  public AutoDriver(MotorControl motorControl, float ft, boolean ifForward) {
    this.ft = ft;
    this.ifForward = ifForward;
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_subsystem = motorControl;
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    encoderCount = m_subsystem.getEncoderCount();
isFinished = false;
  }

  /**
   * This is based on driving the robot 5 feet and recording the encoder count.
   * This is inaccurate compared to calculating the encoder count.
   */
  @Override
  public void execute() {
    //38340
    int direction = 0;
    if(ifForward){
      direction = 1;
    } else {
      direction = -1;
    }
    if ((m_subsystem.getEncoderCount() <= encoderCount + ((38340)/3 * Math.abs(ft))) || m_subsystem.getEncoderCount() >= encoderCount - ((38340)/3 * Math.abs(ft))){
      m_subsystem.drive(-direction * 0.6,0);
    } else {
      isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.drive(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
