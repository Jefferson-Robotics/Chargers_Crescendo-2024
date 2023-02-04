// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OpenMV extends SubsystemBase {
  /** Creates a new OpenMV. */
  SerialPort openMV;
  public OpenMV() {
    openMV = new SerialPort(9600, SerialPort.Port.kUSB);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}