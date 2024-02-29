// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.BufferedInputStream;
import java.io.BufferedOutputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.util.Map;
import java.util.Scanner;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.RecordPlaybackConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Onboarder;
import frc.robot.subsystems.Shooter;

public class playBack extends CommandBase {
  /** Creates a new playBack. */
  private DriveSubsystem swerveController;
  private boolean isFinished;
  private Onboarder onboarder;
  private Shooter shooter;
  private Boolean onRed = false;

  private FileInputStream fileInput;
  private BufferedInputStream bufferedInput;
  private ObjectInputStream objectInput;
  private SendableChooser<File> recSelector;

  private double[] data;
  private double controlLeftY;
  private double controlLeftX;
  private double controlRightX;
  private double onboarderSpeed;
  private double shooterSpeed;

  public playBack(DriveSubsystem swerveController, Onboarder onboarder, Shooter shooter, XboxController controller, SendableChooser<File> RecSelector, Boolean onRed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerveController = swerveController;
    this.onboarder = onboarder;
    this.shooter = shooter;

    this.recSelector = RecSelector;
    this.onRed = onRed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      isFinished = false;
      try {
          fileInput = new FileInputStream(recSelector.getSelected()); //File output stream
          bufferedInput = new BufferedInputStream(fileInput); // buffer output
          objectInput = new ObjectInputStream(bufferedInput); // object output
          System.out.println("Playback Started with: " + recSelector.getSelected());
        
        } catch (IOException e) {
          System.out.println("Failed to read file: ");
          e.printStackTrace();
      }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    try {
      data = (double[]) objectInput.readObject();
      if (!(objectInput.available() > 0)) {
        isFinished = true;
      }
    } catch (Exception e) {
      isFinished = true;
      System.out.println("Failed to playback: ");
      e.printStackTrace();
    }

    controlLeftY = data[0];
    controlLeftX = data[1];
    controlRightX = data[2];
    onboarderSpeed = data[3];
    shooterSpeed = data[3];

    if (onRed) {
      //controlLeftY *= 1;
      controlLeftX *= -1;
      controlRightX *= -1;
    }

    this.swerveController.drive(
      controlLeftY,
      controlLeftX,
      controlRightX,
      true, true
    );
    onboarder.setSpeed(onboarderSpeed);
    shooter.shoot(shooterSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    try {
      objectInput.close();
      bufferedInput.close();
      fileInput.close();
    } catch (IOException e) {
      System.out.println("Failed to end playback: ");
      e.printStackTrace();
    }
  
    this.swerveController.drive(
      0,
      0,
      0,
      true, true
    );
    onboarder.setSpeed(0);
    shooter.shoot(0);
  }
  

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /*
    try {
      if (!(objectInput.available() > 0)) {
        return true;
      } else {
        return false;
      }
      
    } catch (Exception e) {
      return true;
    }
    */
    return isFinished;
  }
}
