// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {
  /** Creates a new Claw. */
  private TalonSRX Gripper = new TalonSRX(4);
  
  private DigitalInput open = new DigitalInput(4);
  private DigitalInput cube = new DigitalInput(6);
  private DigitalInput cone = new DigitalInput(5);
  
  


  public Claw() {}

  public void setSpeed(double speedClaw) {
    Gripper.set(TalonSRXControlMode.PercentOutput, -speedClaw);
    if (!open.get() && speedClaw < 0) {
      Gripper.set(ControlMode.PercentOutput, 0);
    } 
  }
  public boolean isNotOpen() {
    return(open.get());
  }
  public boolean isNotCube() {
    return(cube.get());
  }
  public boolean isNotCone() {
    return(cone.get());
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
