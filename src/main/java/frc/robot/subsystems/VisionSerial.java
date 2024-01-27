// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSerial extends SubsystemBase {
  /** Creates a new VisionSerial. */

  int centerX;
  int centerY;
  int tagWidth;
  int distance;
  int tagID;
  String[] cameraData;
  SerialPort camera;
  public VisionSerial() {
    camera = new SerialPort(115200, SerialPort.Port.kUSB1);
  }

  public String readDataStream() {
    cameraData = camera.readString().split(",", 5);
    return Arrays.toString(cameraData);
  }

  public int getCenterX() {
    return Integer.parseInt(cameraData[0]);
  }

  public int getCenterY() {
    return Integer.parseInt(cameraData[1]);
  }

  public int getTagWitdh() {
    return Integer.parseInt(cameraData[2]);
  }

  public int getDistance() {
    return Integer.parseInt(cameraData[3]);
  }

  public int getTagID() {
    return Integer.parseInt(cameraData[4]);
  }

  @Override
  public void periodic() {
    //System.out.println(this.readDataStream());
    this.readDataStream();
    System.out.println("" + this.getCenterX() + this.getCenterY() + this.getTagWitdh() + this.getDistance() + this.getTagID());
    //System.out.println();
    // This method will be called once per scheduler run
  }
}
