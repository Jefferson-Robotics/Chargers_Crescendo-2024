// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.FileWriter;
import java.io.IOException;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PWM_Motors;

public class record extends CommandBase {
  /** Creates a new recPlay. */
  private PWM_Motors recControl;
  private Joystick lShaft;
  private Joystick rShaft;
  private String recFile;
  private String name = "rec001";
  private FileWriter rFile;
  public record(PWM_Motors recControl, Joystick lShaft, Joystick rShaft) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.recControl = recControl;
    this.lShaft = lShaft;
    this.rShaft = rShaft;
    addRequirements(recControl);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    recFile = "/home/lvuser/" + name + ".txt";
    try {
          rFile = new FileWriter(recFile);
          System.out.println("File created: " + rFile);
          
        } catch (IOException e) {
          System.out.println("An error occurred.");
          e.printStackTrace();
    }
    }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    try {
      rFile.append(String.valueOf(lShaft.getY()) + String.valueOf(rShaft.getY()) + "\n");
    } catch (IOException e) {
      System.out.println("An error occurred.");
      e.printStackTrace();
    }
    this.recControl.drive(lShaft.getY(), rShaft.getY());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    try {
      rFile.close();
    } catch (IOException e) {
      System.out.println("An error occurred.");
      e.printStackTrace();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
