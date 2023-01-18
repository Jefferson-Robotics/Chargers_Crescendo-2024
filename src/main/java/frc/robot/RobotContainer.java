// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.commands.joystickControl;
import frc.robot.subsystems.Can_Motors;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PWM_Motors;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.xboxControl;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.turnNinty;
import frc.robot.commands.record;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private Joystick leftShaft = new Joystick(0);
  private Joystick rightShaft = new Joystick(1);
  private XboxController controller = new XboxController(2);
  private Can_Motors CAN = new Can_Motors();
  private PWM_Motors PWM = new PWM_Motors();
  private turnNinty turn = new turnNinty(CAN);
  private record recBt = new record(PWM, leftShaft, rightShaft);

  //private Reader reader = new Reader(m_motorcontrol);
  private joystickControl joystick = new joystickControl(PWM, leftShaft, rightShaft);
  private xboxControl xbox = new xboxControl(CAN, controller);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //m_motorcontrol.setDefaultCommand(new Reader(m_motorcontrol)/*new Drive(m_motorcontrol, controller)*/);
    CAN.setDefaultCommand(xbox);
    PWM.setDefaultCommand(joystick);
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
    JoystickButton ninty = new JoystickButton(leftShaft, 7);
    ninty.whenPressed(turn);
    JoystickButton rec = new JoystickButton(leftShaft, 11);
    rec.whenPressed(recBt);
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return xbox;
  }
}
