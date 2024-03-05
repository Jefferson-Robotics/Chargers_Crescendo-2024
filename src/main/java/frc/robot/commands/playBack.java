// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.File;
import java.io.IOException;
import java.util.Scanner;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.NoteActuator;
import frc.robot.subsystems.Onboarder;
import frc.robot.subsystems.Shooter;

public class playBack extends Command {
  /** Creates a new playback. */
  private DriveSubsystem swerveController;
  private Onboarder onboarder;
  private Shooter shooter;
  private NoteActuator noteActuator;
  private GenericEntry onRed;
  private SendableChooser<File> recSelector;

  private double controlLeftY;
  private double controlLeftX;
  private double controlRightX;
  private double onboarderSpeed;
  private double shooterSpeed;
  private double rollerSpeed;
  private double actuateSpeed;
  private double liftSpeed;

  private File rFile;
  private Scanner sc;

  public playBack(DriveSubsystem swerveController, Onboarder onboarder, Shooter shooter, NoteActuator noteActuator, XboxController controller, SendableChooser<File> RecSelector, GenericEntry alliancebox) {
    this.swerveController = swerveController;
    this.onboarder = onboarder;
    this.shooter = shooter;
    this.noteActuator = noteActuator;
    

    this.recSelector = RecSelector;
    this.onRed = alliancebox;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveController, onboarder, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    try {
      rFile = new File(recSelector.getSelected().toString());
      sc = new Scanner(rFile);
      System.out.println("Playback Started with: " + recSelector.getSelected());

    } catch (IOException e) {
      System.out.println("Failed to read file: ");
      e.printStackTrace();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    String cLine = sc.nextLine();
    String[] currentArray = cLine.split(",", 6);
    controlLeftY = Double.valueOf(currentArray[0]);
    controlLeftX = Double.valueOf(currentArray[1]);
    controlRightX = Double.valueOf(currentArray[2]);
    onboarderSpeed = Double.valueOf(currentArray[3]);
    shooterSpeed = Double.valueOf(currentArray[4]);
    rollerSpeed = Double.valueOf(currentArray[5]);
    actuateSpeed = Double.valueOf(currentArray[6]);
    liftSpeed = Double.valueOf(currentArray[7]);

    if (!onRed.getBoolean(true)) {
      //controlLeftY *= 1; F&B
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

    noteActuator.setRoller(rollerSpeed);
    noteActuator.actuate(actuateSpeed);
    noteActuator.extendLift(liftSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sc.close();
    this.swerveController.drive(
      0,
      0,
      0,
      true, true
    );
    onboarder.setSpeed(0);
    shooter.shoot(0);
    noteActuator.setRoller(0);
    noteActuator.actuate(0);
    noteActuator.extendLift(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (sc.hasNextLine()) {
      return false;
    } else {
      return true;
    }
  }
}
