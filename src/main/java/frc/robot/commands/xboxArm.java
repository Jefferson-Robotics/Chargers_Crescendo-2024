// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.armSystem;

public class xboxArm extends CommandBase {
  /** Creates a new xboxArm. */
  private armSystem armControl;
  private XboxController controller;
  public xboxArm(armSystem armControl, XboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.armControl = armControl;
    this.controller = controller;
    addRequirements(armControl);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armControl.setSpeed(.7 * controller.getRightY(),.7 * controller.getLeftY());
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