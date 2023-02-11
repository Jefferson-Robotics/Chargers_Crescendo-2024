// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.FileWriter;
import java.io.IOException;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.CANMotorControl;

public class rec extends CommandBase {
  /** Creates a new recPlay. */
  private CANMotorControl recControl;
  private Joystick leftShaft;
  private Joystick rightShaft;
  private String recFile;
  private String name = "rec003";
  private FileWriter rFile;
  public rec(CANMotorControl recControl, Joystick leftShaft, Joystick rightShaft) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.recControl = recControl;
    this.leftShaft = leftShaft;
    this.rightShaft = rightShaft;
    //this.name = name;
    addRequirements(recControl);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    recFile = "/home/lvuser/" + name + ".txt";
    try {
          rFile = new FileWriter(recFile);
          System.out.println("File created: " + rFile);
          
        } catch (IOException e) {
          System.out.println("An error occurred.");
          e.printStackTrace();
    }
    }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    try {
      rFile.append(String.valueOf(-leftShaft.getY()) + "," + String.valueOf(rightShaft.getX()) + "\n");
    } catch (IOException e) {
      System.out.println("An error occurred.");
      e.printStackTrace();
    }
    this.recControl.drive(-leftShaft.getY(), rightShaft.getX());
    SmartDashboard.putNumber("Current recording value of Left Stick", -leftShaft.getY());
    SmartDashboard.putNumber("Current recording value of Right Stick", rightShaft.getX());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    try {
      rFile.close();
    } catch (IOException e) {
      System.out.println("An error occurred.");
      e.printStackTrace();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
