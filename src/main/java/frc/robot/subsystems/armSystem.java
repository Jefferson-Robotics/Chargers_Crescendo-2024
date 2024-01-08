// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import pabeles.concurrency.ConcurrencyOps.Reset;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class armSystem extends SubsystemBase {
  /** Creates a new armSystem. */
  private TalonSRX botLeft = new TalonSRX(7);
  private TalonSRX botRight = new TalonSRX(8);
  private Encoder encoderB = new Encoder(0,1);
  private TalonSRX topLeft = new TalonSRX(6);
  private TalonSRX topRight = new TalonSRX(5);
  private Encoder encoderT = new Encoder(2,3);

  private ComplexWidget bottomEncoderBox;
  private ComplexWidget topEncoderBox;
  private boolean isDocked = true;
  private GenericEntry dockedBox;

  public armSystem(ShuffleboardTab tab) {
    botLeft.setNeutralMode(NeutralMode.Brake);
    botRight.setNeutralMode(NeutralMode.Brake);
    topLeft.setNeutralMode(NeutralMode.Brake);
    topRight.setNeutralMode(NeutralMode.Brake);

    bottomEncoderBox = tab.add("Bottom Encoder", encoderB).withWidget(BuiltInWidgets.kEncoder);
    topEncoderBox = tab.add("Top Encoder", encoderT).withWidget(BuiltInWidgets.kEncoder);

    dockedBox = tab.add("Docked Position", isDocked)
    .withWidget(BuiltInWidgets.kBooleanBox)
    .withProperties(Map.of("colorWhenTrue","orange","colorWhenFalse","grey")).getEntry();
  }
  public void setSpeedBottom(double speedB) {
    botLeft.set(TalonSRXControlMode.PercentOutput, speedB);
    botRight.set(TalonSRXControlMode.PercentOutput, speedB);
  }
  public void setSpeedTop(double speedT) {
    topLeft.set(TalonSRXControlMode.PercentOutput, speedT);
    topRight.set(TalonSRXControlMode.PercentOutput, speedT);
  }

  public double getArmEncoderBottom(){
    return encoderB.get();
  }
  
  public double getArmEncoderTop(){
    return encoderT.get();
  }

  public boolean isFullyDocked() {
    if ((getArmEncoderBottom() < 20 && getArmEncoderBottom() > -10) && (getArmEncoderTop() < 20 && getArmEncoderTop() > -40)) {
      return true;
    } else {
      return false;
    }
  }

  public void resetEncoder() {
    encoderB.reset();
    encoderT.reset();
  }


  public boolean moveBottom(double speed, double finalPos) {
    if (finalPos - getArmEncoderBottom() > -1 * Constants.encoderMargin && finalPos - getArmEncoderBottom() <  Constants.encoderMargin) {
      setSpeedBottom(0);
      return (true);
    } else if (finalPos - getArmEncoderBottom() > 0) {
      setSpeedBottom(-1 * speed);
    } else if (finalPos - getArmEncoderBottom() < 0) {
      setSpeedBottom(speed);
    }
    return (false);
  }
  
  public boolean moveTop(double speed, double finalPos) {
    if (finalPos - getArmEncoderTop() > -1 * Constants.encoderMargin && finalPos - getArmEncoderTop() <  Constants.encoderMargin) {
      setSpeedTop(0);
      return (true);
    } else if (finalPos - getArmEncoderTop() > 0) {
      setSpeedTop(-1 * speed);
    } else if (finalPos - getArmEncoderTop() < 0) {
      setSpeedTop(speed);
    }
    return (false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    isDocked = isFullyDocked();

    dockedBox.setBoolean(isDocked);
    /*
   //System.out.println("Bottom Encoder ------ " + encoderB.get());
   //System.out.println("Top Encoder --------- " + encoderT.get());
    */
  }
}
