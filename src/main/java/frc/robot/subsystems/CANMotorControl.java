// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;

import org.ejml.equation.Variable;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SPI;


public class CANMotorControl extends SubsystemBase {
  /** Creates a new MotorControl. */
  private WPI_TalonFX lmaster = new WPI_TalonFX(10);
  private WPI_TalonFX rmaster = new WPI_TalonFX(11);
  private WPI_TalonFX lslave = new WPI_TalonFX(12);
  private WPI_TalonFX rslave = new WPI_TalonFX(13);
  private AHRS ahrs;
  private DifferentialDrive drive;
  private ShuffleboardTab tab;
  private GenericEntry shuffleboardInput;
  public CANMotorControl(ShuffleboardTab  tab) {
    ahrs = new AHRS(SPI.Port.kMXP); 
    lslave.follow(lmaster);
    rslave.follow(rmaster);
    drive = new DifferentialDrive(lmaster,rmaster);
    //this.tab = tab;
    //shuffleboardInput = tab.add("SpeedInput", BuiltInWidgets.kTextView).getEntry();
    
    rslave.configOpenloopRamp(0.25);
    rmaster.configOpenloopRamp(0.25);
    lslave.configOpenloopRamp(0.25);
    lmaster.configOpenloopRamp(0.25);
    
    
  }

  public void drive(double y,double x){
    drive.arcadeDrive(x, -1 * y);
  }
  public double getEncoderCount(){
    return lslave.getSelectedSensorPosition();
  }

  public double getAngle(){
    return ahrs.getAngle();
  }
  public double getAngleY(){
    return ahrs.getRoll();
  }
  public double getAltitude(){
    return ahrs.getAltitude();
  }
  public void resetGyro(){
    ahrs.reset();
  }
  public void resetEncoder(){
    //lslave.
  }
  public double getAccer(){
    return ahrs.getWorldLinearAccelY();
  }

  @Override
  public void periodic() {
    // System.out.println(getEncoderCount());
    // This method will be called once per scheduler run
    //graph.setDoubleArray(data.toArray(new Double[0]));
    System.out.println("Angle:" + getAngleY());
    System.out.println("Accer: " + getAccer());
    System.out.println("MOTORS:       " + getEncoderCount());
    //Double.parseDouble(shuffleboardInput.getString("-1"));


  }
}
