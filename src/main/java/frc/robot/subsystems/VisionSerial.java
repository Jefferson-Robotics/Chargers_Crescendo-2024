// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.StringTokenizer;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSerial extends SubsystemBase {
  /** Creates a new VisionSerial. */

  double translateX;
  double translateY;
  double[] noValue = {-1,-1,-1,-1};
  double[][] tagData = {
    {-1,-1,-1,-1}, // First tag
    {-1,-1,-1,-1}, // Second tag
    {-1,-1,-1,-1}, // Third tag
    {-1,-1,-1,-1}, // Fourth tag
  };
  // Tag Offset Distance lookup table for Crescendo 2024
  // Access april tag via index Meters
  double[] tagOffsetTable = {
    -1,       //ID  0 - N/A
    0.5, //ID  1 - Blue Source Right
    0.5, //ID  2 - Blue Source Left
    1.5, //ID  3 - Red Speaker Right
    1.5, //ID  4 - Red Speaker Left
    0.5, //ID  5 - Red Amp
    0.5, //ID  6 - Blue Amp
    1.5, //ID  7 - Blue Speaker Right
    1.5, //ID  8 - Blue Speaker Left
    0.5, //ID  9 - Red Source Right
    0.5, //ID 10 - Red Source Left
    0.5, //ID 11 - Red Stage
    0.5, //ID 12 - Red Stage
    0.5, //ID 13 - Red Stage
    0.5, //ID 14 - Blue Stage
    0.5, //ID 15 - Blue Stage
    0.5  //ID 16 - Blue Stage
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
    this.readDataStream();
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
  public double getAngle(int tagID) {
    return getData(tagID)[1];
  }
  public double getTagRotation(int tagID) {
    return getData(tagID)[2];
  }
  public int getTagID(int tagID) {
    return (int) getData(tagID)[3];
  }

  public double getTranslateX(int tagID, double offsetDistance) {
    translateX = getDistance(tagID) * Math.cos(getAngle(tagID)) + getDistance(tagID) * Math.cos(getAngle(tagID));
    //translateX =  
    return translateX;
  }
  public double getTranslateY(int tagID, double offsetDistance) {
    translateY = -1 * getDistance(tagID) * Math.sin(getTagRotation(tagID));
    //translateY = ;
    return translateY;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    this.readDataStream();
    if (tagData[0][0] != -1) {
      //System.out.println("Distance: " + tagData[0][0] + " | Rotation: " + tagData[0][1] + " | Tag Angle: " + tagData[0][2] + " | TagID: " + tagData[0][3]);
    }
    //this.readDataStream();
    //System.out.println("CenX: " + centerX + " | CenY: " + centerY + " | TagW: " + tagWidth + " | Dist: " + distance + " | TagID: " + tagID);
  }
}
