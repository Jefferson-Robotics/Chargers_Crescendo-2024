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
  private boolean activated;
  public NavXAuto() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    activated = false;
    initAngle = m_control.getAngleY();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    

    if(initAngle+5 >= m_control.getAngleY()){
      m_control.drive(1,0);
    } else if(activated == false){
    }
    System.out.println(m_control.getAngleY());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
