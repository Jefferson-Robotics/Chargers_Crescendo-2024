// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class NoteActuator extends SubsystemBase {
  /** Creates a new NoteActuator. */
  private boolean open = false;
  private boolean actuated = false;
  private double rollerSpeed = 0;
  private double actuateSpeed = 0;
  private double liftSpeed = 0;
  

  //private WPI_TalonSRX scissorLift = new WPI_TalonSRX(Constants.NoteAcuatorConstants.kScissorLiftCanID);
  private WPI_TalonSRX liftMotor = new WPI_TalonSRX(Constants.NoteAcuatorConstants.kScissorLiftCanID);
  private WPI_TalonSRX actuator = new WPI_TalonSRX(Constants.NoteAcuatorConstants.kAcuatorCanID);
  private WPI_TalonSRX roller = new WPI_TalonSRX(Constants.NoteAcuatorConstants.kRollerCanID);

  private DigitalInput openPosition = new DigitalInput(0);
  private DigitalInput actuatedPosition = new DigitalInput(1);


  public NoteActuator() {}

  public void setRoller(double power) {
    this.rollerSpeed = power;
  }

  public void actuate(double power) {
    this.actuateSpeed = power;
  }

  public void extendLift(double power) {
    this.liftSpeed = power;
  }

  public double getRollerSpeed() {
    return this.rollerSpeed;
  }
  public double getActuateSpeed() {
    return this.actuateSpeed;
  }
  public double getLiftSpeed() {
    return this.liftSpeed;
  }

  public boolean getOpenPosition() {
    return open;
  }
  public boolean getClosePosition() {
    return actuated;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    open = this.openPosition.get();
    actuated = this.actuatedPosition.get();

    actuator.set(ControlMode.PercentOutput, this.actuateSpeed);
    roller.set(ControlMode.PercentOutput, this.rollerSpeed);
    liftMotor.set(ControlMode.PercentOutput, this.liftSpeed);
  }
}
