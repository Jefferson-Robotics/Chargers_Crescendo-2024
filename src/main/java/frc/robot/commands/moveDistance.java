// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CANMotorControl;

public class moveDistance extends CommandBase {
  /** Creates a new moveDistance. */
  private CANMotorControl drive;
  private double feet;
  private double speed;
  private boolean isDone;
  private double startPos;
  private double currentPos;
  private double endPos;
  public moveDistance(CANMotorControl drive, double feet, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.feet = feet;
    this.speed = speed;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isDone = false;
    startPos = drive.getEncoderCount();
    endPos = (13656 * feet) + startPos;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentPos = drive.getEncoderCount();

    //13656 per foot

    if (endPos - currentPos <= 1000 && endPos - currentPos >= -1000) {
      isDone = true;
    } else if (endPos - currentPos > 1000) {
      drive.drive(speed, 0);
    } else if (endPos - currentPos < 1000) {
      drive.drive(-1 * speed, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
