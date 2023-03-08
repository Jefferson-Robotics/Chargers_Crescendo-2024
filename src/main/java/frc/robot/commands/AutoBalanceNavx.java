// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CANMotorControl;

public class AutoBalanceNavx extends CommandBase {
  /** Creates a new NavXAuto. */
  private CANMotorControl m_control;
  private double drive;
  private boolean isFinished;
  private int stage;
  
  public AutoBalanceNavx(CANMotorControl motorControl) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_control = motorControl;
    addRequirements(motorControl);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stage = 0;
    drive = 0.7;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_control.getAngleY() < -10) {
      stage = 1;
    }
    if (stage == 1) {
      //drive = (2/Math.PI) * Math.atan(Math.min(Math.max(m_control.getAngleY()/10,-45), 45));
      drive = Math.tan(Math.toRadians(Math.min(Math.max(-1 * m_control.getAngleY(),-11),11))*7) / 8;
      /*
      if (-1 < m_control.getAngleY() && m_control.getAngleY() < 1) {
        isFinished = true;
      }
      */
    }
    System.out.println("Drive: "+drive);
    m_control.drive(Math.min(Math.max(drive,-1), 1),0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_control.drive(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
