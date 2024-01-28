// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;
import java.util.StringTokenizer;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSerial extends SubsystemBase {
  /** Creates a new VisionSerial. */

  int centerX = -1;
  int centerY = -1;
  int tagWidth = -1;
  int distance = -1;
  int tagID = -1;
  String cameraData;
  SerialPort camera;
  public VisionSerial() {
    camera = new SerialPort(115200, SerialPort.Port.kUSB1);
  }

  public void readDataStream() {
    this.cameraData= camera.readString();
    StringTokenizer tokenizer = new StringTokenizer(cameraData.trim(), ",");
    if(tokenizer.countTokens() > 3){
      centerX = Integer.parseInt(tokenizer.nextToken());
      centerY = Integer.parseInt(tokenizer.nextToken());
      tagWidth = Integer.parseInt(tokenizer.nextToken());
      distance = Integer.parseInt(tokenizer.nextToken());
      tagID = Integer.parseInt(tokenizer.nextToken());
    }
  }

  public int getCenterX() {
    return centerX;
  }
  public int getCenterY() {
    return centerY;
  }
  public int getTagWidth() {
    return tagWidth;
  }
  public int getDistance() {
    return distance;
  }
  public int getTagID() {
    return tagID;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    this.readDataStream();
    System.out.println("CenX: " + centerX + " | CenY: " + centerY + " | TagW: " + tagWidth + " | Dist: " + distance + " | TagID: " + tagID);
  }
}
