// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.lang.Double;

public class OpenMV extends SubsystemBase {
  /** Creates a new Camerea. */
  SerialPort camera;

  public OpenMV() {
    try{
      camera = new SerialPort(115200, Port.kUSB);
      } catch(Exception e){
        System.out.println("FUCK YOU");
      }
  }

  //x,height;width:distance
  public double cach(int n) {
    double ret = 0;
    String s = camera.readString();

    try{
      if (n == 1) {
        ret = Double.parseDouble(s.substring(0, s.indexOf(",")));
      } else if (n == 2) {
        ret = Double.parseDouble(s.substring(s.indexOf(",")+1, s.indexOf(":")));
      } else if (n == 3) {
        ret = Double.parseDouble(s.substring(s.indexOf(":")+1, s.indexOf("'")));
      } else if (n == 4) {
        ret = Double.parseDouble(s.substring(s.indexOf("'")+1, s.indexOf(";")));
      } else if (n == 5) {
        ret = Double.parseDouble(s.substring(s.indexOf(";")+1, s.indexOf("-")));
      } else if (n == 6) {
        ret = Double.parseDouble(s.substring(s.indexOf("-")+1, s.indexOf("$")));
      } else if (n == 7) {
        ret = Double.parseDouble(s.substring(s.indexOf("$")+1, s.indexOf("_")));
      } else if (n == 8) {
        ret = Double.parseDouble(s.substring(s.indexOf("_")+1));
      }
    } catch(Exception e) {
      ret = -2.0;
    }

    return ret;
  }

  public double getPosX() {
    return cach(1);
  }

  public double getHeight() {
    return cach(2);
  }

  public double getWidth() {
    return cach(3);
  }

  public double getDis() {
    return cach(4);
  }

  public double getCubePosX() {
    return cach(5);
  }

  public double getCubeHeight() {
    return cach(6);
  }

  public double getCubeWidth() {
    return cach(7);
  }

  public double getCubeDis() {
    return cach(8);
  }

  @Override
  public void periodic() {
    //System.out.println(camera.readString());
    // This method will be called once per scheduler run
  }
}

