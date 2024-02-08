// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSerial;

public class AutoTargetAlign extends Command {
  VisionSerial camera;
  DriveSubsystem swerve;
  private boolean isFinished;
  private double tagDistance;
  private double tagRotation;
  private double tagAngle;

  private double translateX;
  private double translateY;
  private double controlRotate;

  /** Creates a new AutoTargetAlign. */
  public AutoTargetAlign() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
    translateX = 0;
    translateY = 0;
    controlRotate = 0;

    swerve.drive(0, 0, 0,  false, false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    
    if (tagDistance != -1) {
      tagDistance = tagDistance / 100; // cm to m

      translateX = tagDistance * Math.cos(tagRotation);
      translateX -= 0.5;
      translateY = -tagDistance * Math.sin(tagRotation);

      System.out.println(tagAngle);
      controlRotate = tagAngle;
    }
    System.out.println("TransX: " + translateX + " | TransY: " + translateY + " | Rotate: " + ((controlRotate / Math.PI) * 180));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.drive(0, 0, 0, false, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
