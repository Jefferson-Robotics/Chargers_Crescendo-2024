// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.File;
import java.io.IOException;
import java.util.Scanner;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CANMotorControl;

public class Reader extends CommandBase {
  /** Creates a new Reader. */
  private File mFile;
  private Scanner myReader;
  private CANMotorControl m_subsystem;

  public Reader(CANMotorControl motorControl) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_subsystem = motorControl;
    
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.drive(0, 0);
    try {
      mFile = new File("C:\\Users\\Chargers\\Documents\\Test.txt");
      myReader = new Scanner(mFile);
      System.out.println("Good");
    } catch (IOException e) {
      System.out.println("FileNotFound");
      try {
        mFile.createNewFile();
        System.out.println("Good");
      } catch (IOException b) {
        System.out.println(mFile);
      }
    }
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.drive(0, 0);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.drive(0, 0);
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return false;
  }
}
