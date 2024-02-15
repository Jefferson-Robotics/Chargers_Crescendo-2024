// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private double topPower = 0;
  private double bottomPower = 0;

  private WPI_TalonSRX topMotor = new WPI_TalonSRX(0);
  private WPI_TalonSRX bottomMotor = new WPI_TalonSRX(0);

  public Shooter() {}

  public void Shoot(double power) {
    this.topPower = power;
    this.bottomPower = -power;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    this.topMotor.set(ControlMode.PercentOutput, this.topPower);
    this.bottomMotor.set(ControlMode.PercentOutput, this.bottomPower);
  }
}
