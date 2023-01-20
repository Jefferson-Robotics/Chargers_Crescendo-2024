// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.Xbox;
import frc.robot.subsystems.MotorControl;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.AutoDriver;
import frc.robot.commands.AutoMove;
import frc.robot.commands.Reader;
import frc.robot.commands.AutoTurn;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private XboxController controller = new XboxController(3);
  private MotorControl m_motorcontrol = new MotorControl();
  //private Reader reader = new Reader(m_motorcontrol);

  private AutoMove move = new AutoMove(m_motorcontrol);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //m_motorcontrol.setDefaultCommand(new Reader(m_motorcontrol)/*new Drive(m_motorcontrol, controller)*/);
    m_motorcontrol.setDefaultCommand(new Xbox(m_motorcontrol, controller));
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return move;
  }
}
