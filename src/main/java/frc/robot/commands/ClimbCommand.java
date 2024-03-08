// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;

public class ClimbCommand extends Command {
  /** Creates a new Climb. */
  private final Climb climb;
  private final XboxController xbox;
  public ClimbCommand(Climb climb, XboxController xbox) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climb = climb;
    this.xbox = xbox;
    addRequirements(climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Extended: " + climb.getExtendedPosition());
    System.out.println("Rest Position: " + climb.getRestPosition());
    if(!climb.getRestPosition()&&90<xbox.getPOV()&&xbox.getPOV()<270){
      climb.setSpeed(-1);
      System.out.println("BOTTOM HIT ");
    }else if(!climb.getExtendedPosition()&&(0<=xbox.getPOV()&&xbox.getPOV()<90) || (270<xbox.getPOV())){
      climb.setSpeed(1);
      System.out.println("TOP HIT ");
    }else{
      climb.setSpeed(0);
    }
    
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
