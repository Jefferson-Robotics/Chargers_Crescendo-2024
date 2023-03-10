// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CANMotorControl;
import java.lang.Math;
import java.util.Currency;

public class AutoBTimed extends CommandBase {
  /** Creates a new AutoB. */
  private CANMotorControl m_control;
  private double drive;
  private boolean isFinished;
  private double startTime;
  private double endTime;
  private int stage;
  private double angle;
  public AutoBTimed(CANMotorControl motorControl) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_control = motorControl;
    addRequirements(motorControl);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = 0;
    endTime = 0;
    stage = 0;
    isFinished = false;
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    angle = m_control.getAngleY()*-1;
    endTime = Timer.getFPGATimestamp();
    if (endTime >= startTime + 2) {
      if (angle > 0) {
        m_control.drive(0.1*Math.pow(angle, 1/3) + 0.012 *angle,0);
      } else {
        m_control.drive(0.1*Math.pow(angle-20, 1/3) + 0.007 *(angle-20),0);
      }
    } else {
      m_control.drive(0.5,0);
    }

    System.out.println("-------Stage-------" + stage);
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
