// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;

public class talonmotor extends SubsystemBase {
  /** Creates a new talonmotor. */
  private double speed;
  private TalonFX motor = new TalonFX(10);
  public talonmotor() {

  }
  public double getSpeed(){
    return speed;
  }
  public void setSpeed(double speed){
    this.speed = speed;
    motor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
