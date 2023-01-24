// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.MotorControl;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.io.Console;

import edu.wpi.first.wpilibj.XboxController;

public class Xbox extends CommandBase {
  /** Creates a new Drive. */
  private MotorControl m_subsystem;
  private XboxController controller;

  public Xbox(MotorControl motorControl, XboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_subsystem = motorControl;
    this.controller = controller;
    addRequirements(m_subsystem);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.drive(-controller.getLeftY(), controller.getRightX());
    System.out.println(m_subsystem.getAngleY());
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