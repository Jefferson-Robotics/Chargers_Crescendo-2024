// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

public class MotorControl extends SubsystemBase {
  /** Creates a new MotorControl. */
  private WPI_TalonFX lmaster = new WPI_TalonFX(10);
  private WPI_TalonFX rmaster = new WPI_TalonFX(11);
  private WPI_TalonFX lslave = new WPI_TalonFX(12);
  private WPI_TalonFX rslave = new WPI_TalonFX(13);
  
  private AHRS ahrs;

  private DifferentialDrive drive;

  public MotorControl() {
    ahrs = new AHRS(SPI.Port.kMXP); 
    lslave.follow(lmaster);
    rslave.follow(rmaster);
    drive = new DifferentialDrive(rmaster,lmaster);
  }

  public void drive(double y,double x){
    drive.arcadeDrive(x*.5, y*.5);
  }
  public double getEncoderCount(){
    return lslave.getSelectedSensorPosition();
  }

  public double getAngle(){
    return ahrs.getAngle();
  }
  public double getAngleY(){
    return ahrs.getPitch();
  }

  public void resetGyro(){
    ahrs.reset();
  }
  public void resetEncoder(){
    //lslave.
  }

  @Override
  public void periodic() {
    // System.out.println(getEncoderCount());
    System.out.println(getAngleY());
    // This method will be called once per scheduler run
    //System.out.println(getAngle());
  }
}
