// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NoteActuator;

public class ScissorOuttake extends Command {
  private boolean isFinished;

  private double initTime = 0;
  private double endingTime = 0;
  private double stopTime = 1;

  /** Creates a new ScissorOuttake. */
  NoteActuator noteActuator;
  public ScissorOuttake(NoteActuator noteActuator) {
    this.noteActuator = noteActuator;
    addRequirements(noteActuator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
    initTime = Timer.getFPGATimestamp();
    endingTime = Timer.getFPGATimestamp() + stopTime;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    noteActuator.actuate(.4);

    initTime = Timer.getFPGATimestamp();

    if (initTime >= endingTime) {
      isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    noteActuator.actuate(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
