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
  private double finalPosTop;
  private double finalPosBottom;
  private double curPosBottom;
  private double curPosTop;
  private double stateB;
  private double stateT;
  private boolean bottomDone;
  private boolean topDone;
  /** Creates a new moveEncodeThird. */
  public moveEncodeThird(armSystem control, double finalPosBottom, double finalPosTop) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.control = control;
    this.finalPosBottom = finalPosBottom;
    this.finalPosTop = finalPosTop;
    addRequirements(control);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stateB = 0;
    stateT = 0;
    bottomDone = false;
    topDone = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    curPosBottom = control.getArmEncoderBottom();
    curPosTop = control.getArmEncoderTop();

    if (stateB == 0) {
      if (control.moveBottom(1, Constants.bottomArmEncoderVertical)) {
        stateB = 1;
        stateT = 1;
      } 
    } else if (stateB == 1) {
      if (control.moveBottom(1, finalPosBottom)) {
        stateB = 2;
        bottomDone = true;
      } 
    } 

    if (stateT == 1) {
      if (control.moveTop(1, -340)) {
        stateT = 2;
      }
    } else if (stateT == 2) {
      if (control.moveTop(0.5, -390)) {
        stateT = 3;
      }
    } else if (stateT == 3) {
      if (control.moveTop(0.2, finalPosTop)) {
        stateT = 4;
        topDone = true;
      }
    }
    

    /*if (state == 0) {
      if (control.moveBottom(0.8, finalPosBottom)) {
        state = 1;
      }
    } else if (state == 1) {
      if (control.moveTop(0.7, -340)) {
        state = 2;
      }
    } else if (state == 2) {
      if (control.moveTop(0.35, -390)) {
        state = 3;
      }
    } else if (state == 3) {
      if (control.moveTop(0.25, finalPosTop)) {
        isDone = true;
      }
    } else if (state == 4) {
      if (clawSystem.isNotOpen()) {
        clawSystem.setSpeed(-1);
      } else {
        clawSystem.setSpeed(0);
        isDone = true;
      }*/
    
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
    return (topDone && bottomDone);
  }
}
