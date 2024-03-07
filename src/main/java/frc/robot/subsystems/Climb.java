// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase {
  /** Creates a new Climb. */
  private double speed = 0;
  private boolean rest = false;
  private boolean extended = false;

  private DigitalInput restPosition = new DigitalInput(Constants.ClimbConstants.kRestLimitID);
  private DigitalInput extendPosition = new DigitalInput(Constants.ClimbConstants.kExtendLimitID);
  private WPI_TalonSRX onboardMotor = new WPI_TalonSRX(Constants.ClimbConstants.kClimbMotorID);

  public Climb() {}

  public void setSpeed(double speed) {
    this.speed = speed;
  }

  public boolean getRestPosition() {
    return rest;
  }
  public boolean getExtendedPosition() {
    return extended;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    rest = this.restPosition.get();
    extended = this.extendPosition.get();

    //if (rest == true || extended == true) {
   //   onboardMotor.set(ControlMode.PercentOutput, 0);
   // } else {
      onboardMotor.set(ControlMode.PercentOutput, speed);
   // }
  }
}
