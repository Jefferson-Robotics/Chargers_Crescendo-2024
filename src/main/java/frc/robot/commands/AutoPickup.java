// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Onboarder;

public class AutoPickup extends Command {
  /** Creates a new NoteAuto. */
  private final Camera camera;
  private DriveSubsystem drive;
  private final Onboarder NoteSystem;
  private int x;
  private int y;
  private boolean done = false;
  public AutoPickup(DriveSubsystem drive,Onboarder noteSystem,Camera cameraSystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    NoteSystem = noteSystem;
    camera = cameraSystem;
    this.drive=drive;
    addRequirements(cameraSystem);
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    done=false;
    x=0;
    y=0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    camera.readDataStream();
    x=camera.getX();
    y=camera.getY();
    System.out.println(x);
    System.out.println(y);
    if(y<240){
      if(x<-25){
        drive.drive(.2, .2, 0, false, true);
      }else if(25<x){
        drive.drive(-.2, .2, 0, false, true);
      }else{
        drive.drive(0, .2, 0, false, true);
      }
    }else{
      drive.drive(0, 0, 0, false, true);
      done=true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.drive(0, 0, 0, false, true);
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
