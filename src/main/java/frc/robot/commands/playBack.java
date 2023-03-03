// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.File;
import java.util.Scanner;
import java.io.IOException;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CANMotorControl;


public class playBack extends CommandBase {
  /** Creates a new playBack. */
  private CANMotorControl playControl;
  private double lPlay;
  private double rPlay;
  private String name;
  private boolean isBlue;
  private File rFile;
  private Scanner sc;
  public playBack(CANMotorControl playControl, boolean isBlue, String name) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.playControl = playControl;
    this.isBlue = isBlue;
    this.name = name;
    addRequirements(playControl);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    try {
      rFile = new File("/home/lvuser/" + name + ".txt");
      sc = new Scanner(rFile);
    } catch (IOException e) {
      System.out.println("An error occurred.");
      e.printStackTrace();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    String cLine = sc.nextLine();
    String[] currentArray = cLine.split(",", 8);
    lPlay = Double.valueOf(currentArray[0]);
    rPlay = Double.valueOf(currentArray[1]);
    if (!isBlue) {
      rPlay = -1 * rPlay;
    }
    this.playControl.drive(lPlay, rPlay);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sc.close();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (sc.hasNextLine()) {
      return false;
    } else {
      return true;
    }
  }
}
