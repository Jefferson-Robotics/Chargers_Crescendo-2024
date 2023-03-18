// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.armSystem;

public class AutoDock extends CommandBase {
  /** Creates a new AutoDock. */
  private armSystem arm;
  private boolean isDone;
  private double bottomPos;
  private double topPos;
  private double state;
  public AutoDock(armSystem arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isDone = false;
    state = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    bottomPos = arm.getArmEncoderBottom();
    topPos = arm.getArmEncoderTop();

    if (state == 0) {
      if (arm.moveTop(0.6, 570)) {
        state = 1;
      }
    } else if (state == 1) {
      if (arm.moveBottom(0.5, -135)) {
        isDone = true;
      }
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
