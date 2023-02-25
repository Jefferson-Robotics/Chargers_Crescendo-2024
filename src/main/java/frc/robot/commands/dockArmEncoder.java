// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.armSystem;

public class dockArmEncoder extends CommandBase {
  /** Creates a new dockArmEncoder. */
  private armSystem control;
  private boolean isDone;
  private double state;
  private double curPosBottom;
  private double curPosTop;
  public dockArmEncoder(armSystem control) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.control = control;
    addRequirements(control);
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
    curPosBottom = control.getArmEncoderBottom();
    curPosTop = control.getArmEncoderTop();

    if (state == 0) {

      if (Constants.bottomArmEncoderVertical - curPosBottom > -1 * Constants.encoderMargin && Constants.bottomArmEncoderVertical - curPosBottom <  Constants.encoderMargin) {
        state = 1;
        control.setSpeedBottom(0);
      } else if (Constants.bottomArmEncoderVertical - curPosBottom > 0) {
        control.setSpeedBottom(-0.5);
      } else if (Constants.bottomArmEncoderVertical - curPosBottom < 0) {
        control.setSpeedBottom(0.5);
      }

    } else if (state == 1) {

      if (curPosTop > -1 * Constants.encoderMargin && curPosTop <  Constants.encoderMargin) {
        state = 2;
        control.setSpeedTop(0);
      } else if (curPosTop < 0) {
        control.setSpeedTop(-0.5);
      } else if (curPosTop > 0) {
        control.setSpeedTop(0.5);
      }

    } else if (state == 2) {

      if (curPosBottom > -1 * Constants.encoderMargin && curPosBottom <  Constants.encoderMargin) {
        isDone = true;
        control.setSpeedBottom(0);
      } else if (curPosBottom < 0) {
        control.setSpeedBottom(-0.5);
      } else if (curPosBottom > 0) {
        control.setSpeedBottom(0.5);
      }

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    control.setSpeedBottom(0);
    control.setSpeedTop(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}

