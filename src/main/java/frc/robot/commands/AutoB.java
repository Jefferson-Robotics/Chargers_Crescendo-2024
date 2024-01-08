// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CANMotorControl;
import java.lang.Math;

public class AutoB extends CommandBase {
  /** Creates a new AutoB. */
  private CANMotorControl m_control;
  private double drive;
  private boolean isFinished;
  private int stage;
  private double angle;
  public AutoB(CANMotorControl motorControl) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_control = motorControl;
    addRequirements(motorControl);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stage = 0;
    isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    angle = m_control.getAngleY()*-1;
    if (stage == 0){
        m_control.drive(0.5,0);
        if(angle>10) {
          stage = 1;
        }
    } else if (stage == 1) {
        m_control.drive(0.2,0);
        if(angle<8) {
          stage = 2;
        }
    } else if (stage == 2) {
        m_control.drive(0.5,0);
        if(angle>11) {
          stage = 3;
        }
    } else if (stage == 3) {
        m_control.drive(0.4,0);
        if(angle>13) {
          stage=4;
        }
    } else if (stage == 4) {
      if (angle > 0) {
        m_control.drive(0.1*Math.pow(angle, 1/3) + 0.011 *angle,0);
      } else {
        m_control.drive(0.1*Math.pow(angle-20, 1/3) + 0.010 *(angle-20),0);
      }
    }

    //System.out.println("-------Stage-------" + stage);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_control.drive(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
