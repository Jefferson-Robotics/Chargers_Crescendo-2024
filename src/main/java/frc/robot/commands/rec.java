// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.FileWriter;
import java.io.IOException;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.CANMotorControl;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.armSystem;

public class rec extends CommandBase {
  /** Creates a new recPlay. */
  private CANMotorControl recControl;
  private armSystem arm;
  private Claw claw;
  private Joystick leftShaft;
  private Joystick rightShaft;
  private XboxController controller;
  private JoystickButton[] buttons;
  private String recFile;
  private String name = "rec003";
  private FileWriter rFile;
  public rec(CANMotorControl recControl, armSystem arm, Claw claw, Joystick leftShaft, Joystick rightShaft, XboxController controller, JoystickButton[] buttons) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.recControl = recControl;
    this.arm = arm;
    this.claw = claw;
    this.leftShaft = leftShaft;
    this.rightShaft = rightShaft;
    this.controller = controller;
    this.buttons = buttons;
    //this.name = name;
    addRequirements(recControl, arm, claw);
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
      rFile.append(String.valueOf(-leftShaft.getY()) + "," + String.valueOf(rightShaft.getX()) + "," + String.valueOf(buttons[0].getAsBoolean()) + "," + String.valueOf(buttons[1].getAsBoolean()) + "," + String.valueOf(buttons[2].getAsBoolean()) + "," + String.valueOf(buttons[3].getAsBoolean()) + "," + String.valueOf(buttons[4].getAsBoolean()) + "," + String.valueOf(buttons[5].getAsBoolean()) + "\n");
      System.out.println(String.valueOf(-leftShaft.getY()) + "," + String.valueOf(rightShaft.getX()) + "," + String.valueOf(buttons[0].getAsBoolean()) + "," + String.valueOf(buttons[1].getAsBoolean()) + "," + String.valueOf(buttons[2].getAsBoolean()) + "," + String.valueOf(buttons[3].getAsBoolean()) + "," + String.valueOf(buttons[4].getAsBoolean()) + "," + String.valueOf(buttons[5].getAsBoolean()) + "\n");
    } catch (IOException e) {
      System.out.println("An error occurred.");
      e.printStackTrace();
    }
    this.recControl.drive(-leftShaft.getY(), rightShaft.getX());
    SmartDashboard.putNumber("Current recording value of Left Stick", -leftShaft.getY());
    SmartDashboard.putNumber("Current recording value of Right Stick", rightShaft.getX());
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
