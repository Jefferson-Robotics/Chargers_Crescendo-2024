// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NoteActuator;

public class LiftScissorLift extends Command {
  /** Creates a new LiftScissorLift. */
  NoteActuator noteActuator;
  XboxController controller;
  public LiftScissorLift(NoteActuator noteActuator,  XboxController controller) {
    this.noteActuator = noteActuator;
    this.controller = controller;
    addRequirements(noteActuator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    noteActuator.extendLift(-controller.getRightY());
    /*
    if(noteActuator.getScissorLimitSwitch()){
       noteActuator.extendLift(-controller.getRightY());
    } else {
       noteActuator.extendLift(0);
    }
    */
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
