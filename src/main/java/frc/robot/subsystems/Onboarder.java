// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Onboarder extends SubsystemBase {
  /** Creates a new Onboarder. */

  private double speed = 0;

  private DigitalInput kIntakeBeam = new DigitalInput(Constants.OnboarderConstants.kIntakeBeam);
  private DigitalInput kOutakeBeam = new DigitalInput(Constants.OnboarderConstants.kOutakeBeam);
  private WPI_VictorSPX onboardMotor = new WPI_VictorSPX(Constants.OnboarderConstants.konboardMotorcanID);
  
  public Onboarder() {}

  public boolean intake(){
    return !this.kIntakeBeam.get();
  }
  public boolean outTake(){
    return !this.kOutakeBeam.get();
  }

  public double getSpeed() {
    return this.speed;
  }
  public void setSpeed(double speed) {
    this.speed = -speed;
  }

  @Override
  public void periodic() {
    onboardMotor.set(ControlMode.PercentOutput, speed);
    // This method will be called once per scheduler run
  }
}
