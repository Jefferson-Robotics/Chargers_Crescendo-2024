// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import pabeles.concurrency.ConcurrencyOps.Reset;

public class armSystem extends SubsystemBase {
  /** Creates a new armSystem. */
  private TalonSRX botLeft = new TalonSRX(7);
  private TalonSRX botRight = new TalonSRX(8);
  private Encoder encoderB = new Encoder(0,1);
  private TalonSRX topLeft = new TalonSRX(6);
  private TalonSRX topRight = new TalonSRX(5);
  private Encoder encoderT = new Encoder(2,3);


  public armSystem() { 
    botLeft.setNeutralMode(NeutralMode.Brake);
    botRight.setNeutralMode(NeutralMode.Brake);
    topLeft.setNeutralMode(NeutralMode.Brake);
    topRight.setNeutralMode(NeutralMode.Brake);
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

  public void resetEncoder() {
    encoderB.reset();
    encoderT.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    System.out.println("Bottom Encoder ------ " + encoderB.get());
    System.out.println("Top Encoder --------- " + encoderT.get());
  }
}
