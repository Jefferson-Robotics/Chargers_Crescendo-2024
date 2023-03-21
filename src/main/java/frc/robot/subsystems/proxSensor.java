// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class proxSensor extends SubsystemBase {
  /** Creates a new proxSensor. */
  AnalogInput proxSensor = new AnalogInput(0);

  public proxSensor() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    System.out.println("------------------------------------------ PROXIMITY SENSOR " + proxSensor.getValue());
  }
}
