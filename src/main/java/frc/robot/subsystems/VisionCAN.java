// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.hal.CANData;
import edu.wpi.first.wpilibj.CAN;

public class VisionCAN extends SubsystemBase {
  private CAN visionCan = new CAN(45);
  private CANData visionCanData;
  /** Creates a new VisionCAN. */
  public VisionCAN() {
    visionCan.readPacketLatest​(45,
visionCanData) {

}
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
