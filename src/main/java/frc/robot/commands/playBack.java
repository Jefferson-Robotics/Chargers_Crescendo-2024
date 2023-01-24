// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.File;
import java.util.Scanner;
import java.io.IOException;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PWM_Motors;


public class playBack extends CommandBase {
  /** Creates a new playBack. */
  private PWM_Motors playControl;
  private double lPlay;
  private double rPlay;
  private String name = "rec003";
  private File rFile;
  private Scanner sc;
  public playBack(PWM_Motors playControl) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.playControl = playControl;
    //this.name = name;
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
    String[] currentArray = cLine.split(",", 2);
    lPlay = Double.valueOf(currentArray[0]);
    rPlay = Double.valueOf(currentArray[1]);
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
