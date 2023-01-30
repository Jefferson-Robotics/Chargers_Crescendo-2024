// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MotorControl;

public class AutoTurn extends CommandBase {
  /** Creates a new Turn. */

  private MotorControl subsystem;

  private double initAngle = 0;

  private boolean isFinished = false;

  private int degrees = 0;

  private boolean ifRight;

  public AutoTurn(MotorControl subsystem, int degrees, boolean ifRight) {
    this.subsystem = subsystem;
    this.degrees = degrees;
    this.ifRight = ifRight;
    addRequirements(subsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    subsystem.resetGyro();
    initAngle = Math.abs(subsystem.getAngle());
  }

  /**
   * 87 is an estimate of going to 90 degrees to account for overshoot.
   */
  @Override
  public void execute() {
    int direction = 0;
    if(ifRight){
      direction = 1;
    } else {
      direction = -1;
    }
    //subsystem.drive(0,(.8 - (subsystem.getAngle() - initAngle) / 90 * .3));
    subsystem.drive(0, direction * 0.4 * Math.pow(degrees - (subsystem.getAngle() - initAngle), .124364));
    if (Math.abs(subsystem.getAngle()) > initAngle + degrees || Math.abs(subsystem.getAngle())  < initAngle - degrees){
      isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subsystem.drive(0,0);
    subsystem.resetGyro();
    isFinished = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}