// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.File;
import java.util.Map;
import java.util.Scanner;
import java.io.IOException;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.subsystems.CANMotorControl;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.armSystem;

public class playBack extends CommandBase {
  /** Creates a new playBack. */
  private CANMotorControl playControl;
  private armSystem arm;
  private Claw claw;
  private ShuffleboardTab tab;
  private GenericEntry textbox;
  private GenericEntry allianceTog;

  private double lPlay;
  private double rPlay;
  private double tPlay;
  private double bPlay;
  private double oPlay;
  private double cPlay;

  private File rFile;
  private Scanner sc;
  public playBack(CANMotorControl playControl, armSystem arm, Claw claw, ShuffleboardTab tab) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.playControl = playControl;
    this.arm = arm;
    this.claw = claw;

    this.tab = tab;
    textbox = tab.add("Recording", "default value")
      .withWidget(BuiltInWidgets.kTextView).getEntry();

    allianceTog = tab.add("Switch Alliance", false)
      .withWidget(BuiltInWidgets.kToggleSwitch).getEntry();

    addRequirements(playControl, arm, claw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    try {
      rFile = new File("/home/lvuser/" + textbox.getString("rec003") + ".txt");
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
    String[] currentArray = cLine.split(",", 6);
    lPlay = Double.valueOf(currentArray[0]);
    rPlay = Double.valueOf(currentArray[1]);
    tPlay = Double.valueOf(currentArray[2]);
    bPlay = Double.valueOf(currentArray[3]);
    oPlay = Double.valueOf(currentArray[4]);
    cPlay = Double.valueOf(currentArray[5]);

    if (allianceTog.getBoolean(false)) {
      rPlay = -1 * rPlay;
    }
    this.playControl.drive(lPlay, rPlay);
    this.arm.setSpeedTop(tPlay);
    this.arm.setSpeedBottom(bPlay);
    this.claw.setSpeed(cPlay - oPlay);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sc.close();
    playControl.drive(0, 0);
    arm.setSpeedTop(0);
    arm.setSpeedBottom(0);
    claw.setSpeed(0);
    new ScheduleCommand(new AutoBalanceNavx(playControl)).schedule();

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
