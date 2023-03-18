// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CANMotorControl;

public class newBalance extends CommandBase {
  /** Creates a new newBalance. */
  private CANMotorControl drive;
  private double angle;
  private boolean isDone;
  private double speed;
  public newBalance(CANMotorControl drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isDone = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    angle = drive.getAngleY();

    if (angle <= 0) {
      speed = -0.2*Math.pow(angle, 1/3) - 0.03 *angle;
    } else {
      speed = -0.2*Math.pow(angle-5, 1/3) - 0.03 *(angle-5);
    }

    drive.drive(speed, 0);
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
