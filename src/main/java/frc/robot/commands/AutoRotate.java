// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSerial;

public class AutoRotate extends Command {
  /** Creates a new AutoRotate. */
  private boolean isFinished = false;
  private VisionSerial camera;
  private DriveSubsystem swerve;
  private boolean rightTurn;

  private double initAngle;
  private double degrees;


  public AutoRotate(VisionSerial camera, DriveSubsystem swerve, int degrees, boolean rightTurn) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.camera = camera;
    this.swerve = swerve;
    this.degrees = degrees;
    this.rightTurn = rightTurn;
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerve.resetGyro();
    initAngle = Math.abs(swerve.getAngle());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //System.out.println(isFinished);
    double direction = 0;
    if(rightTurn){
      direction = 0.5;
    } else {
      direction = -0.5;
    }
    //subsystem.drive(0,(.8 - (subsystem.getAngle() - initAngle) / 90 * .3));
    swerve.drive(
      0,
      0,
      ( direction * 0.4 * Math.pow(degrees - (swerve.getAngle() - initAngle), .124364)),
      false,
      false);

    if (Math.abs(swerve.getAngle()) > initAngle + degrees || Math.abs(swerve.getAngle())  < initAngle - degrees){
      isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.drive(0,0,0,true,true);
    isFinished = false;
    rightTurn = false;
    initAngle = 0;
    degrees = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println(isFinished);
    return isFinished;
  }
}
