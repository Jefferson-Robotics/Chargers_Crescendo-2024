// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.commands.joystickControl;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.AutoBalanceNavx;
import frc.robot.commands.AutoDriver;
import frc.robot.commands.AutoMove;
import frc.robot.commands.Xbox;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
//import frc.robot.commands.turnNinty;
import frc.robot.commands.rec;
import frc.robot.commands.xboxArm;
import frc.robot.commands.playBack;
import frc.robot.subsystems.CANMotorControl;
import frc.robot.subsystems.OpenMV;
import frc.robot.subsystems.armSystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

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
  private CANMotorControl mControl = new CANMotorControl();
  private rec recCommand = new rec(mControl, leftShaft, rightShaft);
  private playBack playB = new playBack(mControl);
  private armSystem arm = new armSystem();


  //Camera access with a search.
  //private OpenMV camera = new OpenMV();

  //private Reader reader = new Reader(m_motorcontrol);
  private joystickControl joystick = new joystickControl(mControl, leftShaft, rightShaft);
  private Xbox xbox = new Xbox(mControl, controller);
  private xboxArm armControl = new xboxArm(arm, controller);
  private SendableChooser<Command> m_Chooser = new SendableChooser<Command>();



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //m_motorcontrol.setDefaultCommand(new Reader(m_motorcontrol)/*new Drive(m_motorcontrol, controller)*/);
    m_Chooser.setDefaultOption("Auto Balance", new AutoBalanceNavx(mControl));
    m_Chooser.addOption("Right Wall Auto (AutoMove)", new AutoMove(mControl));
    SmartDashboard.putData("Autonomous mode chooser", m_Chooser);
    mControl.setDefaultCommand(xbox);
    arm.setDefaultCommand(armControl);
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
    //JoystickButton ninty = new JoystickButton(leftShaft, 7);
    //ninty.whenPressed(turn);
    JoystickButton recButton = new JoystickButton(rightShaft, 11);
    JoystickButton recButton2 = new JoystickButton(rightShaft, 10);
    recButton.onTrue(recCommand.until(recButton2));
    JoystickButton playBack = new JoystickButton(rightShaft, 6);
    playBack.onTrue(playB);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_Chooser.getSelected();
  }
}
