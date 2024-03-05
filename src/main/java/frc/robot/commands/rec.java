// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.RecordPlaybackConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.NoteActuator;
import frc.robot.subsystems.Onboarder;
import frc.robot.subsystems.Shooter;

public class rec extends Command {
  /** Creates a new rec. */
  private DriveSubsystem swerveController;
  private Onboarder onboarder;
  private Shooter shooter;
  private NoteActuator noteActuator;
  private XboxController controller;

  private FileWriter rFile;
  private SendableChooser<File> RecSelector;
  private GenericEntry recFileName;
  private File recordFile;
  private int fileCount = 0;

  private double controlLeftY;
  private double controlLeftX;
  private double controlRightX;
  private double onboarderSpeed;
  private double shooterSpeed;
  private double rollerSpeed;
  private double actuateSpeed;
  private double liftSpeed;

  public rec(DriveSubsystem swerveController, Onboarder onboarder, Shooter shooter, NoteActuator noteActuator, XboxController controller, SendableChooser<File> RecSelector, GenericEntry FileName) {
    this.controller = controller;
    
    this.swerveController = swerveController;
    this.onboarder = onboarder;
    this.shooter = shooter;
    this.noteActuator = noteActuator;
    
    this.RecSelector = RecSelector;
    this.recFileName = FileName;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveController);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    try {
        recordFile = new File(RecordPlaybackConstants.kRecordDirectory,recFileName.getString("rec"+fileCount)+"."+RecordPlaybackConstants.kFileType);
        rFile = new FileWriter(recordFile);
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
    rollerSpeed = noteActuator.getRollerSpeed();
    actuateSpeed = noteActuator.getActuateSpeed();
    liftSpeed = noteActuator.getLiftSpeed();
    try {
      rFile.append(
        String.valueOf(controlLeftY) + "," +
        String.valueOf(controlLeftX) + "," +
        String.valueOf(controlRightX) + "," +
        String.valueOf(onboarderSpeed) + "," +
        String.valueOf(shooterSpeed) + "," +
        String.valueOf(rollerSpeed) + "," +
        String.valueOf(actuateSpeed) + "," +
        String.valueOf(liftSpeed) + "," +
        "\n"
      );
    } catch (IOException e) {
     //System.out.println("An error occurred.");
      e.printStackTrace();
    }
    this.swerveController.drive(
      controlLeftY,
      controlLeftX,
      controlRightX,
      true, true
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    try {
      rFile.close();
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
