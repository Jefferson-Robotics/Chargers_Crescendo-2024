// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {
  /** Creates a new Claw. */
  private TalonSRX Gripper = new TalonSRX(4);
  
 // private DigitalInput open = new DigitalInput(4);
  //private DigitalInput cube = new DigitalInput(6);
  //private DigitalInput cone = new DigitalInput(5);
  
  private DutyCycleEncoder encoder = new DutyCycleEncoder(7);
  private ShuffleboardTab tab;
  private boolean conePosition = false;
  private boolean cubePosition = false;
  private boolean clawConnection = false;
  private boolean clawOpen = false;
  private GenericEntry coneBooleanBox;
  private GenericEntry cubeBooleanBox;
  private GenericEntry openBooleanBox;
  private GenericEntry clawBooleanBox;


  public Claw(ShuffleboardTab tab) {
    this.tab = tab;
    //Closed

    //Cone
    coneBooleanBox = tab.add("Cone Position",conePosition)
    .withWidget(BuiltInWidgets.kBooleanBox)
    .withProperties(Map.of("colorWhenTrue","yellow","colorWhenFalse","grey")).getEntry();
    //Inbetween
    //Cube
    cubeBooleanBox = tab.add("Cube Position",cubePosition)
    .withWidget(BuiltInWidgets.kBooleanBox)
    .withProperties(Map.of("colorWhenTrue","purple","colorWhenFalse","grey")).getEntry();
    //Open
    openBooleanBox = tab.add("Open Position",clawOpen)
    .withWidget(BuiltInWidgets.kBooleanBox)
    .withProperties(Map.of("colorWhenTrue","green","colorWhenFalse","grey")).getEntry();
    //Connection
    clawBooleanBox = tab.add("Claw Connection",clawConnection)
    .withWidget(BuiltInWidgets.kBooleanBox)
    .withProperties(Map.of("colorWhenTrue","green","colorWhenFalse","red")).getEntry();
  }

  public void setSpeed(double speedClaw) {
    if ((encoderPos() > 0.74 && speedClaw < 0) || (encoderPos() < 0.25 && speedClaw > 0)) {
      Gripper.set(ControlMode.PercentOutput, 0);
    } else {
      Gripper.set(TalonSRXControlMode.PercentOutput, -speedClaw);
    }
  }
  public double encoderPos() {
    return(encoder.getAbsolutePosition());
  }
  public boolean encoderWorking() {
    return encoder.isConnected();
  }
  /*
   * public boolean isNotOpen() {
    return(open.get());
  }
  public boolean isNotCube() {
    return(cube.get());
  }
  public boolean isNotCone() {
    return(cone.get());
  }
   */
  


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double position = encoder.getAbsolutePosition();
    clawConnection = encoderWorking();

    if(position < 0.22){
      conePosition = false;
      cubePosition = false;
      clawOpen = false;
    } else if(position < 0.28){
      conePosition = true;
      cubePosition = false;
      clawOpen = false;
    }else if (position < 0.52) {
      conePosition = false;
      cubePosition = false;
      clawOpen = false;
    }else if(position < 0.58){
      conePosition = false;
      cubePosition = true;
      clawOpen = false;
    } else if (position < 0.71) {
      cubePosition = false;
      conePosition = false;
      clawOpen = false;
    } else if (position < 0.77) {
      conePosition = false;
      cubePosition = false;
      clawOpen = true;
    } else{
      conePosition = false;
      cubePosition = false;
      clawOpen = true;
    }
    coneBooleanBox.setBoolean(conePosition);
    cubeBooleanBox.setBoolean(cubePosition);
    openBooleanBox.setBoolean(clawOpen);
    clawBooleanBox.setBoolean(clawConnection);
    //System.out.println("Claw Encoder <<<<<<< " + encoder.getAbsolutePosition());
  }
}
