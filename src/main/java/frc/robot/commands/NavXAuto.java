// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MotorControl;

public class NavXAuto extends CommandBase {
  /** Creates a new NavXAuto. */
  private MotorControl m_control;
  private double initAngle;
  private double drive;
  private boolean isFinished;
  public NavXAuto(MotorControl motorControl) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_control = motorControl;
    addRequirements(motorControl);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initAngle = m_control.getAngleY();
    isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive = 1/180*(m_control.getAngleY()-initAngle);
    m_control.drive(drive,0);
    
    System.out.println(m_control.getAngleY());
    if(drive<=.3){
      isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
