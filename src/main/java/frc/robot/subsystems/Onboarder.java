// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Onboarder extends SubsystemBase {
  /** Creates a new Onboarder. */

  private double speed = 0;

  private DigitalInput BeamInput = new DigitalInput(Constants.OnboarderConstants.kbeamBreakPort);
  private WPI_TalonSRX onboardMotor = new WPI_TalonSRX(Constants.OnboarderConstants.konboardMotorcanID);

  public Onboarder() {}

  public boolean isTriggered(){
    return !this.BeamInput.get();
  }

  public void intake(double speed) { 
    this.speed = speed;
  }

  public void outtake(double speed) {
    this.speed = -speed;
  }

  @Override
  public void periodic() {
    onboardMotor.set(ControlMode.PercentOutput, speed);
    // This method will be called once per scheduler run
  }
}
