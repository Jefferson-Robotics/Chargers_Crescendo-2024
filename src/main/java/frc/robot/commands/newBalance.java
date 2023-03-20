// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CANMotorControl;
import java.lang.Math;

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

  private void drive(double x, double direction){
    if(direction > 0) {
      drive.drive(x- (Math.abs(drive.getAccer()* .4)), 0);
    } else {
      drive.drive((x * -1) + (Math.abs(drive.getAccer() * .4 )),0);
    }
  } 
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    angle = drive.getAngleY()*-1;
    if (angle < 2 && angle > -2) {
      System.out.println("1");
      drive(.13,angle);
    } else if (angle < 5 && angle > -5) {
      System.out.println("2");
      drive(.17,angle);
    } else if (angle < 10 && angle > -10) {
      System.out.println("3");
      drive(.19,angle);
    } else if (angle < 15 && angle > -15) {
      System.out.println("4");
      drive(.25,angle);
    } else if (angle < 20 && angle > -20) {
      System.out.println("5");
      drive(.24,angle);
    } else {
      System.out.println("6");
      drive.drive(0,0);
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
