// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.FileWriter;
import java.io.IOException;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class rec extends CommandBase {
  /** Creates a new recPlay. */
  private DriveSubsystem swerveController;
  private XboxController controller;
  private String recFile;
  private String recFileName;
  private Integer fileID;
  private FileWriter rFile;

  private double controlLeftY;
  private double controlLeftX;
  private double controlRightX;

  public rec(DriveSubsystem swerveController, XboxController controller, String recFileNameParam, Integer fileIDParam) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerveController = swerveController;
    this.controller = controller;
    this.recFileName = recFileNameParam;
    this.fileID = fileIDParam;
    //this.name = name;
    addRequirements(swerveController);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    recFile = "/home/lvuser/" + recFileName + fileID + ".txt";
    try {
          rFile = new FileWriter(recFile);
         //System.out.println("File created: " + rFile);
          
        } catch (IOException e) {
         //System.out.println("An error occurred.");
          e.printStackTrace();
    }
    }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    controlLeftY = -MathUtil.applyDeadband(controller.getLeftY() * -.5, OIConstants.kDriveDeadband);
    controlLeftX = -MathUtil.applyDeadband(controller.getLeftX() * -.5, OIConstants.kDriveDeadband);
    controlRightX = -MathUtil.applyDeadband(controller.getRightX() * -.5, OIConstants.kDriveDeadband);
    try {
      rFile.append(String.valueOf(controlLeftY) + "," + String.valueOf(controlLeftX) + "," + String.valueOf(controlRightX) + "\n");
    } catch (IOException e) {
     //System.out.println("An error occurred.");
      e.printStackTrace();
    }
    this.swerveController.drive(
      controlLeftY,
      controlLeftX,
      controlRightX,
      true, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    fileID++;
    try {
      rFile.close();
    } catch (IOException e) {
     //System.out.println("An error occurred.");
      e.printStackTrace();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
