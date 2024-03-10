// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Onboarder;
import frc.robot.subsystems.Shooter;

public class PrimeShooter extends Command {
  private boolean isFinished;
  private Shooter shooter;
  private Onboarder onboarder;

  double initTime = 0;
  double endingTime = 0;
  double stopTime = 0.3;

  /** Creates a new PrimeShooter. */
  public PrimeShooter(Shooter shooter, Onboarder onboarder) {
    this.shooter = shooter;
    this.onboarder = onboarder;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, onboarder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
    initTime = Timer.getFPGATimestamp();
    endingTime = Timer.getFPGATimestamp() + stopTime;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    initTime = Timer.getFPGATimestamp();

    /*
    if (initTime >= endingTime) {
      onboarder.setSpeed(0);
      shooter.shoot(1);
    } else {
      onboarder.setSpeed(-0.5);
    }
    */
    if (!onboarder.outTake()) {
      onboarder.setSpeed(0);
      shooter.shoot(1);
    } else {
      onboarder.setSpeed(-0.5);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.shoot(0);
    onboarder.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
