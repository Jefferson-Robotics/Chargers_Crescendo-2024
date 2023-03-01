// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class grab extends CommandBase {
  private int position;
  private Claw claw;
  private boolean isDone;
  /** Creates a new grab. */
  public grab(Claw claw, int position) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.position = position;
    this.claw = claw;
    addRequirements(claw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isDone = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (position == 0) {

      if (claw.isNotOpen()) {
        claw.setSpeed(-1);
      } else {
        isDone = true;
      }
    }
    if (position == 1) {

      if (claw.isNotCube() && !claw.isNotCone()) {
        claw.setSpeed(1);
      } else {
        isDone = true;
      }
    }
    if (position == 2) {

      if (claw.isNotCone()) {
        claw.setSpeed(1);
      } else {
        isDone = true;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    claw.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
