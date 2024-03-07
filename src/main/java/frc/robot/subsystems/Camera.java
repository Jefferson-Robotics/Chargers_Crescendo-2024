// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CameraConstants;

public class Camera extends SubsystemBase {
  /** Creates a new CameraSubsystem. */
  private String[] cameraData = {"0","0"};
  private SerialPort camera = new SerialPort(115200, CameraConstants.kCamera);

  public Camera() {}
  public void readDataStream(){
    try {
      String[] temp = camera.readString().split(",");
      if(temp[0].length()!=0){
        cameraData=temp;
      }
    }
    catch(Exception e) {
      e.printStackTrace();
    }
  }
  public int getX(){
    try{
      return Integer.parseInt(cameraData[0]);
    }catch(Exception e){
      readDataStream();
      return getX();
    }
  }
  public int getY(){
    try{
      return Integer.parseInt(cameraData[1]);
    }catch(Exception e){
      readDataStream();
      return getY();
    }
  };

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
