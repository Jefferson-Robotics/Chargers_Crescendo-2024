// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Can_Motors;

public class turnNinty extends CommandBase {
  /** Creates a new turnNinty. */
  private Can_Motors m_subsystem;
  private double initialPosition;
  private double remainDist;
  final static double kP = .0003;
  private boolean isDone = false;
  public turnNinty(Can_Motors motorControl) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_subsystem = motorControl;
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isDone = false;
    initialPosition = m_subsystem.getSensorPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //4096
    if(initialPosition+1028<=m_subsystem.getSensorPosition()){
      isDone = true;
    }else{
      remainDist = (initialPosition+1028) - m_subsystem.getSensorPosition();

      m_subsystem.drive(kP*remainDist);
    }
    /*subsystem.drive(0, direction * 0.4 * Math.pow(degrees - (subsystem.getAngle() - initAngle), .124364));
    if (Math.abs(subsystem.getAngle()) > initAngle + degrees || Math.abs(subsystem.getAngle())  < initAngle - degrees){
      isFinished = true;*/
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.drive(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}