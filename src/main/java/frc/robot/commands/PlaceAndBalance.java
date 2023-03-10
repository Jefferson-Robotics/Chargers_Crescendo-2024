// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CANMotorControl;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.armSystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceAndBalance extends SequentialCommandGroup {
  /** Creates a new PlaceAndBalance. */
  public PlaceAndBalance(CANMotorControl drive, armSystem arm, Claw claw) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new grab(claw, 1), new moveEncodeThird(arm, claw, -550, -500), new grab(claw, 0), new dockArmEncoder(arm), new AutoBTimedGoobo(drive));
  }
}
