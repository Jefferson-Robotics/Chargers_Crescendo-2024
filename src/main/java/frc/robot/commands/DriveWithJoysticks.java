// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.swing.plaf.TreeUI;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveWithJoysticks extends PIDCommand {
  /** Creates a new DriveWithJoysticks. */
  public DriveWithJoysticks(DriveSubsystem drive, XboxController controller) {
    super(
        // The controller that the command will use
        new PIDController(0, 0, 0),
        // This should return the measurement
        () -> drive.getAngle(),
        // This should return the setpoint (can also be a constant)
        () -> drive.getChosenAngle(),
        // This uses the output
        output -> {
          double xSpeed = -MathUtil.applyDeadband(controller.getLeftY() * -.5, OIConstants.kDriveDeadband);
          double ySpeed = -MathUtil.applyDeadband(controller.getLeftX() * -.5, OIConstants.kDriveDeadband);
          if(!(-MathUtil.applyDeadband(controller.getRightX() * -.5, OIConstants.kDriveDeadband) > 0)){        
            drive.setChosenAngle(output + Constants.DriveConstants.HeadingTurnRate);
          }
          // Use the output here
          drive.drive(xSpeed, ySpeed, output, true, true);
        });
        addRequirements(drive);
    SmartDashboard.putNumber("P", 0);
    SmartDashboard.putNumber("I",0);
    SmartDashboard.putNumber("D",0);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    getController().setPID(SmartDashboard.getNumber("P", 0),SmartDashboard.getNumber("I",0),SmartDashboard.getNumber("D",0));
    return false;
  }
}
