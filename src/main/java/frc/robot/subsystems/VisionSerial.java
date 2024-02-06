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

  double[] noValue = {-1,-1,-1,-1};
  double[][] tagData = {
    {-1,-1,-1,-1}, // First tag
    {-1,-1,-1,-1}, // Second tag
    {-1,-1,-1,-1}, // Third tag
    {-1,-1,-1,-1}, // Fourth tag
  };

  String cameraData;
  SerialPort camera;
  public VisionSerial() {
    camera = new SerialPort(115200, SerialPort.Port.kUSB1);
  }
  public void readDataStream() {
    this.cameraData = camera.readString();
    //System.out.println(this.cameraData);
    StringTokenizer tagSeperater = new StringTokenizer(cameraData.trim(), "|");
    for (int i = 0; i < tagSeperater.countTokens(); i++) {
      StringTokenizer tagDataSeperater = new StringTokenizer(cameraData.trim(), ",");
      if(tagDataSeperater.countTokens() > 4) {
        tagData[i][0] = Double.parseDouble(tagDataSeperater.nextToken());
        tagData[i][1] = Double.parseDouble(tagDataSeperater.nextToken());
        tagData[i][2] = Double.parseDouble(tagDataSeperater.nextToken());
        tagData[i][3] = Integer.parseInt(tagDataSeperater.nextToken());
      }
      tagSeperater.nextToken();
    }
  }

  public double[] getData(int tagID) {
    for (int i = 0; i < 3; i++) {
      if (tagData[i][3] == tagID) {
        return tagData[i];
      }
    }
    return noValue;
  }

  public double getDistance(int tagID) {
    return getData(tagID)[0];
  }
  public double getRotation(int tagID) {
    return getData(tagID)[1];
  }
  public double getTagAngle(int tagID) {
    return getData(tagID)[2];
  }
  public int getTagID(int tagID) {
    return (int) getData(tagID)[3];
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    this.readDataStream();
    //System.out.println("Distance: " + tagData[0][0] + " | Rotation: " + tagData[0][1] + " | Tag Angle: " + tagData[0][2] + " | TagID: " + tagData[0][3]);
  }
}
