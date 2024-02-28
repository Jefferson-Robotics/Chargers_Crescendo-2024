// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.File;
import java.util.Map;
import java.util.Scanner;
import java.io.IOException;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Onboarder;
import frc.robot.subsystems.Shooter;

public class playBack extends CommandBase {
  /** Creates a new playBack. */
  private DriveSubsystem swerveController;
  private Onboarder onboarder;
  private Shooter shooter;
  private ShuffleboardTab tab;
  private GenericEntry textbox;
  private String recFileName;
  private Boolean onRed = false;

  private double controlLeftY;
  private double controlLeftX;
  private double controlRightX;
  private double onboarderSpeed;
  private double shooterSpeed;

  private File rFile;
  private Scanner sc;
  public playBack(DriveSubsystem swerveController, Onboarder onboarder, Shooter shooter, XboxController controller, ShuffleboardTab tab, String recFileNameParam, Boolean onRed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerveController = swerveController;
    this.onboarder = onboarder;
    this.shooter = shooter;
    this.onRed = onRed;

    this.recFileName = recFileNameParam;



    this.tab = tab;
    textbox = tab.add("Recording", "default value")
      .withWidget(BuiltInWidgets.kTextView).getEntry();
    addRequirements(swerveController, onboarder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    try {
      rFile = new File("/home/lvuser/" + recFileName + ".txt");
      sc = new Scanner(rFile);
    } catch (IOException e) {
     //System.out.println("An error occurred.");
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
    sc.close();
    this.swerveController.drive(
      0,
      0,
      0,
      true, true);
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
