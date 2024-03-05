// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private double topPower = 0;
  private double bottomPower = 0;
  private TalonFX topMotor;
  private TalonFX bottomMotor;

  public Shooter() {
    bottomMotor = new TalonFX(7);
    topMotor = new TalonFX(6);
  }

  public void shoot(double power) {
    this.topPower = -power;
    this.bottomPower = power;
  }

  public double getSpeed() {
    return this.topPower;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    this.topMotor.set(topPower);
    this.bottomMotor.set(bottomPower);
  }
}
