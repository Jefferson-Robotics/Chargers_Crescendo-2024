// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CANMotorControl;
import frc.robot.subsystems.OpenMV;
import frc.robot.commands.AutoTurn;
import frc.robot.commands.AutoMove;

public class AutoMoveToPiece extends CommandBase {
  /** Creates a new AutoMoveToPiece. */
  private CANMotorControl m_subsystem;
  private OpenMV camera;
  private boolean aligned;
  private boolean isFinished;
  private double xpos;
  private double distance;

  public AutoMoveToPiece(CANMotorControl motorControl, OpenMV camera) {
    this.m_subsystem = motorControl;
    this.camera = camera;
    addRequirements(m_subsystem, camera);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    aligned = false;
    isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    xpos = camera.getPosX();
    distance = camera.getDis();
    if (!aligned) {
      double turnSpeed = (xpos / 160 - 1) / 2;
      turnSpeed = turnSpeed + 0.2 * Math.signum(turnSpeed);
      m_subsystem.drive(0,turnSpeed);
      if (xpos > 75 && xpos < 85) {// how ever much it has to be aligned
        aligned = true;
        m_subsystem.drive(0,0);
      }
    } else {
      if (distance > 1) {// drive until 1 feet or so from camera// will have to be changed
        m_subsystem.drive(0.6,0);
      } else {
        // This here should be when it trys to grab the piece
        // Remember that it still has to know which piece it is picking up
        isFinished = true;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.drive(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
