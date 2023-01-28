// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class Can_Motors extends SubsystemBase {
  /** Creates a new MotorControl. */
  private WPI_TalonFX Speen = new WPI_TalonFX(6);

  public Can_Motors() {}
  
  public double getSensorPosition(){
    return Speen.getSelectedSensorPosition();
  }

  public void drive(double y){
    Speen.set(y);
  }

  public double getCurrent(){
    return Speen.getSupplyCurrent();
  } 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //System.out.println(getAngle());
    //System.out.println("Current"  + getCurrent());
    //System.out.println("Speed" + Speen.getMotorOutputVoltage());
  }
}
