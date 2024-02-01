// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSerial;

public class CenterOnTarget extends Command {
  /** Creates a new CenterOnTarget. */
  VisionSerial camera;
  DriveSubsystem swerve;
  private boolean isFinished;
  private int cameraX;
  private int cameraDistance;
  private double controlRotate;
  private double controlX;
  private double controlY;

  private double cameraDegrees;
  private double distMidBot;
  private double cameraHFOV;
  private double cameraHFOVRadians;
  private int horizontalRes;
  private int distanceSet;
  private double botRadians;

  public CenterOnTarget(VisionSerial camera, DriveSubsystem swerve) {
    this.camera = camera;
    this.swerve = swerve;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
    controlRotate = 0;
    controlX = 0;
    controlY = 0;

    cameraX = -1;
    distanceSet = 45;
    distMidBot = 26.5;
    cameraHFOV = 70.8;
    cameraHFOVRadians = (cameraHFOV / 360) * 2 * Math.PI;
    horizontalRes = 300;

    swerve.drive(0, 0,0,  true, true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    cameraX = camera.getCenterX(1);
    cameraDistance = camera.getDistance(1);
    if (cameraX != -1 && (cameraX < 140 || cameraX > 160) && (cameraDistance < 70 || cameraDistance > 100)) {
      // Rotate towards AprilTag
      cameraDegrees = (cameraX - 150) * cameraHFOVRadians / horizontalRes;
      botRadians = Math.atan( (Math.sin(cameraDegrees) * cameraDistance)  / (Math.cos(cameraDegrees) * cameraDistance + distMidBot));
      controlRotate = -0.5 * Math.pow(botRadians / (cameraHFOVRadians / 2),3); //Boost rotation power
      
      // Drive towards AprilTag
      controlX = 0.3 * Math.max(-1, Math.min(.2 * Math.pow((double) (cameraDistance - distanceSet) / distanceSet,3), 1));
      /*
      if (cameraDistance < 70) {
        controlX = -0.25;
      } else if (cameraDistance > 100) {
        controlX = 0.2
      }*/
      
    } else if ((cameraX > 140 && cameraX < 160) && (cameraDistance > 70 && cameraDistance < 100)) {
      //isFinished = true;
      //controlX = 0;
      //controlRotate = 0;
    } else {
      controlX = 0;
      controlRotate = 0;
    }
    this.swerve.drive(
      controlX,
      0,
      controlRotate,
      false, true);
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}