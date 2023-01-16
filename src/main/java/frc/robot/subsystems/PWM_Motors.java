// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Talon;

public class PWM_Motors extends SubsystemBase {
  /** Creates a new PWMMoterControl. */
  private Talon lmaster = new Talon(9);
  private Talon rmaster = new Talon(8);
  private Talon lfollow = new Talon(1);
  private Talon rfollow = new Talon(0);
  private MotorControllerGroup left =  new MotorControllerGroup(lmaster, lfollow);
  private MotorControllerGroup right =  new MotorControllerGroup(rmaster, rfollow);
  private DifferentialDrive drive;
  public PWM_Motors() {
    drive = new DifferentialDrive(left,right);
  }
  public void drive(double y,double x){
    drive.tankDrive(x*-.7, y*.7);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
