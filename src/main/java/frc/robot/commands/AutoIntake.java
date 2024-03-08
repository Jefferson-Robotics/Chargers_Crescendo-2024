// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Onboarder;

public class AutoIntake extends Command {
  /** Creates a new AutoIntake. */
  Onboarder onboarder;
  XboxController controller;
  public AutoIntake(Onboarder onboarder, XboxController controller) {
    this.onboarder = onboarder;
    this.controller = controller;
    addRequirements(onboarder);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(controller.getLeftY()) > .1) {
      onboarder.setSpeed(controller.getLeftY());
    } else if(onboarder.outTake()){
      onboarder.setSpeed(0);
    } else if(onboarder.intake()) {
      onboarder.setSpeed(1);
    }else {
      onboarder.setSpeed(0);
    }
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
