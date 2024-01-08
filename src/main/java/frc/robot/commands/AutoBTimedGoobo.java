// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CANMotorControl;
import java.lang.Math;
import java.util.Currency;

public class AutoBTimedGoobo extends CommandBase {
  /** Creates a new AutoB. */
  private CANMotorControl m_control;
  private double speed;
  private boolean isFinished;
  private double startTime;
  private double endTime;
  private int stage;
  private double angle;
  public AutoBTimedGoobo(CANMotorControl motorControl) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_control = motorControl;
    addRequirements(motorControl);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = 0;
    endTime = 0;
    stage = 0;
    isFinished = false;
    startTime = Timer.getFPGATimestamp();
  }

  private void drive(double x, double direction){
    if(direction > 0)
      m_control.drive(x- (Math.abs(m_control.getAccer()* .4)), 0);
    else
      m_control.drive((x * -1) + (Math.abs(m_control.getAccer() * .4 )),0);
  } 
  private void driveWeighted(){
    speed = 0;
    angle = m_control.getAngleY()*-1;
    endTime = Timer.getFPGATimestamp();
    if (endTime >= startTime + 2.1) {
       /*if (Math.abs(m_control.getAccer()) > .25) {
       //System.out.println("1");
        drive(0,angle);
      } else*/
    //    if (angle < 2 && angle > -2) {
    //    //System.out.println("1");
    //     drive(.13,angle);
    //   } else if (angle < 5 && angle > -5) {
    //    //System.out.println("2");
    //     drive(.17,angle);
    //   } else if (angle < 10 && angle > -10) {
    //    //System.out.println("3");
    //     drive(.185,angle);
    //   } else if (angle < 15 && angle > -15) {
    //    //System.out.println("4");
    //     drive(.25,angle);
    //   } else if (angle < 20 && angle > -20) {
    //    //System.out.println("5");
    //     drive(.24,angle);
    //   } else {
    //    //System.out.println("6");
    //     m_control.drive(0,0);
    //   }
    // } else {
    //   m_control.drive(.5,0);
    // }
    if (angle < 2 && angle > -2) {
     //System.out.println("1");
      drive(.13,angle);
    } else if (angle < 5 && angle > -5) {
     //System.out.println("2");
      drive(.17,angle);
    } else if (angle < 10 && angle > -10) {
     //System.out.println("3");
      drive(.2,angle);
    } else if (angle < 15 && angle > -15) {
     //System.out.println("4");
      drive(.25,angle);
    } else if (angle < 20 && angle > -20) {
     //System.out.println("5");
      drive(.3,angle);
    } else {
     //System.out.println("6");
      m_control.drive(0,0);
    }
  } else {
      m_control.drive(.5,0);
    }
  }

  private void driveNoWeight(){
    speed = 0;
    angle = m_control.getAngleY()*-1;
    endTime = Timer.getFPGATimestamp();
    if (endTime >= startTime + 1.7) {
       /*if (Math.abs(m_control.getAccer()) > .25) {
       //System.out.println("1");
        drive(0,angle);
      } else*/
       if (angle < 2 && angle > -2) {
       //System.out.println("1");
        drive(.13,angle);
      } else if (angle < 5 && angle > -5) {
       //System.out.println("2");
        drive(.17,angle);
      } else if (angle < 10 && angle > -10) {
       //System.out.println("3");
        drive(.19,angle);
      } else if (angle < 15 && angle > -15) {
       //System.out.println("4");
        drive(.25,angle);
      } else if (angle < 20 && angle > -20) {
       //System.out.println("5");
        drive(.24,angle);
      } else {
       //System.out.println("6");
        m_control.drive(0,0);
      }
    } else {
      m_control.drive(.6,0);
    }
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveNoWeight();
    //driveWeighted();
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
