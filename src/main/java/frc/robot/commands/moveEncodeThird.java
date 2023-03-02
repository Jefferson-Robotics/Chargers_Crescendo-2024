// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.armSystem;

public class moveEncodeThird extends CommandBase {
  private armSystem control;
  private Claw clawSystem;
  private double finalPosTop;
  private double finalPosBottom;
  private double curPosBottom;
  private double curPosTop;
  private double state;
  private boolean isDone;
  /** Creates a new moveEncodeThird. */
  public moveEncodeThird(armSystem control, Claw clawSystem, double finalPosBottom, double finalPosTop) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.control = control;
    this.clawSystem = clawSystem;
    this.finalPosBottom = finalPosBottom;
    this.finalPosTop = finalPosTop;
    addRequirements(control);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    state = 0;
    isDone = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    curPosBottom = control.getArmEncoderBottom();
    curPosTop = control.getArmEncoderTop();

    if (state == 0) {
      
      if (finalPosBottom - curPosBottom > -1 * Constants.encoderMargin && finalPosBottom - curPosBottom <  Constants.encoderMargin) {
        state = 1;
        control.setSpeedBottom(0);
      } else if (finalPosBottom - curPosBottom > 0) {
        control.setSpeedBottom(-0.5);
      } else if (finalPosBottom - curPosBottom < 0) {
        control.setSpeedBottom(0.5);
      }

    } else if (state == 1) {
      
      if (finalPosTop-curPosTop > -1 * Constants.encoderMargin && finalPosTop-curPosTop < Constants.encoderMargin) {
        state = 2;
      } else if (finalPosTop-curPosTop > 0) {
        control.setSpeedTop(-0.4);
      } else if (finalPosTop-curPosTop < 0) {
        control.setSpeedTop(0.4);
      }
    } else if (state == 2) {
      if (clawSystem.isNotOpen()) {
        clawSystem.setSpeed(-1);
      } else {
        isDone = true;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    control.setSpeedTop(0);
    control.setSpeedBottom(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
