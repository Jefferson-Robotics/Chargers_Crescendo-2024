// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.NoteActuator;
import frc.robot.subsystems.Onboarder;
import frc.robot.subsystems.Shooter;

public class CancelAll extends Command {
  /** Creates a new CancelAll. */
  private boolean isFinished;

  Onboarder onboarder;
  Climb climb;
  DriveSubsystem drive;
  Shooter shooter;
  NoteActuator noteActuator;

  public CancelAll(Onboarder onboarder, Climb climb, DriveSubsystem drive, Shooter shooter, NoteActuator noteActuator) {
    this.onboarder = onboarder;
    this.climb = climb;
    this.drive = drive;
    this.shooter = shooter;
    this.noteActuator = noteActuator;

    addRequirements(onboarder, climb, drive, shooter, noteActuator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    onboarder.setSpeed(0.01);
    climb.setSpeed(0.01);
    drive.drive(
      0.01,
      0.01,
      0.01,
      false,
      false);
    shooter.shoot(0.01);
    noteActuator.setRoller(0.01);
    noteActuator.actuate(0.01);
    noteActuator.extendLift(0.01);

    isFinished = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    onboarder.setSpeed(0);
    climb.setSpeed(0);
    drive.drive(
      0,
      0,
      0,
      false,
      false);
    shooter.shoot(0);
    noteActuator.setRoller(0);
    noteActuator.actuate(0);
    noteActuator.extendLift(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
