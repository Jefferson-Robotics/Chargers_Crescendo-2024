// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class armSystem extends SubsystemBase {
  /** Creates a new armSystem. */
  private TalonSRX left = new TalonSRX(7);
  private TalonSRX right = new TalonSRX(8);
  public armSystem() { 

  }
  public void setSpeed(double speed) {
    left.set(TalonSRXControlMode.PercentOutput, speed);
    right.set(TalonSRXControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
