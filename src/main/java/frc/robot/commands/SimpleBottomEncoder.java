// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.armSystem;

public class SimpleBottomEncoder extends CommandBase {
  /** Creates a new AutoDock. */
  private armSystem arm;
  private boolean isDone;
  private double speed;
  private double finalPos;
  public SimpleBottomEncoder(armSystem arm, double speed, double finalPos) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    this.speed = speed;
    this.finalPos = finalPos;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isDone = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (arm.moveBottom(speed, finalPos)) {
      isDone = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
