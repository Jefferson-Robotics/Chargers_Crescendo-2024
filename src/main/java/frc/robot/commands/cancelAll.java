// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CANMotorControl;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.armSystem;

public class cancelAll extends CommandBase {
  /** Creates a new cancelAll. */
  private CANMotorControl drive;
  private armSystem arm;
  private Claw claw;
  private boolean isDone;
  public cancelAll(CANMotorControl drive, armSystem arm, Claw claw) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.arm = arm;
    this.claw = claw;
    addRequirements(drive, arm, claw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isDone = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.drive(0.01,0.01);
    arm.setSpeedBottom(0.01);
    arm.setSpeedTop(0.01);
    claw.setSpeed(0.01);

    isDone = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
