// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.MotorControl;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Joystick;

public class joystickControl extends CommandBase {
  /** Creates a new Drive. */
  //private CanMotor m_subsystem;
  private MotorControl m_subsystem;
  private Joystick leftShaft;
  private Joystick rightShaft;

  public joystickControl(MotorControl motorControl, Joystick leftShaft, Joystick rightShaft) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_subsystem = motorControl;
    this.leftShaft = leftShaft;
    this.rightShaft = rightShaft;
    addRequirements(m_subsystem);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.drive(leftShaft.getY(), rightShaft.getX());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //print.close();
    m_subsystem.drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}