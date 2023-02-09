// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import pabeles.concurrency.ConcurrencyOps.Reset;

public class armSystem extends SubsystemBase {
  /** Creates a new armSystem. */
  private TalonSRX left = new TalonSRX(7);
  private TalonSRX right = new TalonSRX(8);
  private Encoder encoderB = new Encoder(0,1);
  private DigitalInput stopper = new DigitalInput(2);
  public armSystem() { 

  }
  public void setSpeed(double speedB, double speedT) {
    if (!(stopper.get() && speedB>0)) {
      left.set(TalonSRXControlMode.PercentOutput, speedB);
      right.set(TalonSRXControlMode.PercentOutput, speedB);
    } else {
      left.set(TalonSRXControlMode.PercentOutput, 0);
      right.set(TalonSRXControlMode.PercentOutput, 0);
    }
    System.out.println(stopper.get());
  }
  public double getArmEncoderBottom(){
    return encoderB.get();
  }
  public void resetEncoder() {
    encoderB.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    System.out.println(encoderB.get());
  }
}
