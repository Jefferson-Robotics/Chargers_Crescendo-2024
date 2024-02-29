// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.BufferedOutputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.FilenameFilter;
import java.io.IOException;
import java.io.ObjectOutput;
import java.io.ObjectOutputStream;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.RecordPlaybackConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Onboarder;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class rec extends CommandBase {
  /** Creates a new recPlay. */
  private DriveSubsystem swerveController;
  private Onboarder onboarder;
  private Shooter shooter;
  private XboxController controller;

  private FileOutputStream fileOutput;
  private BufferedOutputStream bufferedOutput;
  private ObjectOutputStream objectOutput;
  private SendableChooser<File> RecSelector;
  private GenericEntry recFileName;
  private File recordFile;
  private int fileCount = 0;

  private double controlLeftY;
  private double controlLeftX;
  private double controlRightX;
  private double onboarderSpeed;
  private double shooterSpeed;

  public rec(DriveSubsystem swerveController, Onboarder onboarder, Shooter shooter, XboxController controller, SendableChooser<File> RecSelector, GenericEntry FileName) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerveController = swerveController;
    this.onboarder = onboarder;
    this.shooter = shooter;

    this.RecSelector = RecSelector;
    this.recFileName = FileName;

    this.controller = controller;
    //this.name = name;
    addRequirements(swerveController);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      try {
          recordFile = new File(RecordPlaybackConstants.kRecordDirectory,recFileName.getString("rec"+fileCount)+"."+RecordPlaybackConstants.kFileType);
          fileOutput = new FileOutputStream(recordFile); //File output stream
          bufferedOutput = new BufferedOutputStream(fileOutput); // buffer output
          objectOutput = new ObjectOutputStream(bufferedOutput); // object output
          System.out.println("Recording at: " + recordFile);
        
        } catch (IOException e) {
          System.out.println("Failed to create file: ");
          e.printStackTrace();
      }
    }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    controlLeftY = -MathUtil.applyDeadband(controller.getLeftY() * -.5, OIConstants.kDriveDeadband);
    controlLeftX = -MathUtil.applyDeadband(controller.getLeftX() * -.5, OIConstants.kDriveDeadband);
    controlRightX = -MathUtil.applyDeadband(controller.getRightX() * -.5, OIConstants.kDriveDeadband);
    onboarderSpeed = onboarder.getSpeed();
    shooterSpeed = shooter.getSpeed();

    double[] recordData = {controlLeftY, controlLeftX, controlRightX, onboarderSpeed, shooterSpeed};

    try {
      objectOutput.writeObject(recordData);
    } catch (IOException e) {
      System.out.println("Failed to start record: ");
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
    try {
      objectOutput.close();
      bufferedOutput.close();
      fileOutput.close();
    } catch (IOException e) {
      System.out.println("Failed to end record: ");
      e.printStackTrace();
    }

    RecSelector.addOption(recordFile.getName(), recordFile);
    fileCount++;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
