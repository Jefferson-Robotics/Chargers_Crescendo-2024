// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.CANMotorControl;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.armSystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class newAuto extends SequentialCommandGroup {
  /** Creates a new newAuto. */
  public newAuto(CANMotorControl drive, armSystem arm, Claw claw) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    //18 point - Place and move -
    //addCommands(new moveEncodeThird(arm, -550, -550), new grab(claw, 0), new dockArmEncoder(arm), new moveDistance(drive, 7, 0.7), new newBalance(drive));
    // 21 point - Place and balance -
    addCommands(new moveEncodeThird(arm, -550, -550), new grab(claw, 0), new dockArmEncoderAuto(arm), new moveDistance(drive, 8, 0.7), new moveDistance(drive, 6.5, 0.4), new moveDistance(drive, -6.5, 0.6), new newBalance(drive));
    //addCommands(new moveEncodeThird(arm, -550, -550), new grab(claw, 0), new SimpleBottomEncoder(arm, 0.8, Constants.bottomFrontStop), new moveDistance(drive, 8, 0.7), new moveDistance(drive, 5.5, 0.4), new moveDistance(drive, -5.5, 0.6), new newBalance(drive));
  }
}
