// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.commands.joystickControl;
import frc.robot.commands.moveDistance;
import frc.robot.commands.moveEncodeThird;
import frc.robot.commands.moveEncoder;
import frc.robot.commands.moveEncoderFront;
import frc.robot.commands.newAuto;
import frc.robot.commands.newBalance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.commands.AutoB;
import frc.robot.commands.AutoBTimedGoobo;
import frc.robot.commands.AutoBalanceNavx;
import frc.robot.commands.AutoDriver;
import frc.robot.commands.AutoMove;
import frc.robot.commands.PlaceAndBalance;
import frc.robot.commands.Xbox;
import frc.robot.commands.cancelAll;
import frc.robot.commands.dockArmEncoder;
import frc.robot.commands.grab;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
//import frc.robot.commands.turnNinty;
import frc.robot.commands.rec;
import frc.robot.commands.resetArmEncoders;
import frc.robot.commands.xboxArm;
import frc.robot.commands.playBack;
import frc.robot.subsystems.CANMotorControl;
import frc.robot.subsystems.OpenMV;
import frc.robot.subsystems.armSystem;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.proxSensor;
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
  private ShuffleboardTab tab = Shuffleboard.getTab("Playback");
  private Joystick leftShaft = new Joystick(0);
  private Joystick rightShaft = new Joystick(1);
  private XboxController controller = new XboxController(2);
  private CANMotorControl mControl = new CANMotorControl(tab);
  private armSystem arm = new armSystem(tab);
  private Claw clawControl = new Claw();
  private proxSensor prox = new proxSensor();
  private rec recCommand;
  private playBack playB = new playBack(mControl, arm, clawControl, tab);
  
  

  //Camera access with a search.
  //private OpenMV camera = new OpenMV();

  //private Reader reader = new Reader(m_motorcontrol);
  private joystickControl joystick = new joystickControl(mControl, leftShaft, rightShaft);
  private Xbox xbox = new Xbox(mControl, controller);
  private xboxArm armControl = new xboxArm(arm, controller, clawControl);
  private SendableChooser<Command> m_Chooser = new SendableChooser<Command>();

  private resetArmEncoders reset = new resetArmEncoders(arm);
  private dockArmEncoder armZero = new dockArmEncoder(arm);
  private moveEncoderFront movePos1 = new moveEncoderFront(arm, clawControl, 90, -80);
  private moveEncoder movePos2 = new moveEncoder(arm, clawControl, 90, -170);
  private moveEncodeThird movePos3 = new moveEncodeThird(arm, clawControl, -550, -500);
  private AutoB balance = new AutoB(mControl);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //m_motorcontrol.setDefaultCommand(new Reader(m_motorcontrol)/*new Drive(m_motorcontrol, controller)*/);
    /*
     * m_Chooser.setDefaultOption("Playback", playB);
    m_Chooser.addOption("Balance", balance);
    m_Chooser.addOption("Playback with Balance", autoPB);
    tab.add("Autonomous", m_Chooser);
     */
    


    mControl.setDefaultCommand(joystick);
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
    JoystickButton cancelCommands = new JoystickButton(rightShaft,2);
    cancelCommands.onTrue(new cancelAll(mControl, arm, clawControl));
    JoystickButton release = new JoystickButton(leftShaft, 1);
    release.onTrue(new grab(clawControl, 0));
    JoystickButton resetArmEncoders1 = new JoystickButton(rightShaft, 10);
    resetArmEncoders1.onTrue(reset);
    JoystickButton resetArmEncoders2 = new JoystickButton(rightShaft, 11);
    resetArmEncoders2.onTrue(reset);


    JoystickButton dockArmButton = new JoystickButton(controller, Button.kB.value);
    dockArmButton.onTrue(armZero);
    JoystickButton frontOneButton = new JoystickButton(controller, Button.kA.value);
    frontOneButton.onTrue(movePos1);
    JoystickButton coneTwoButton= new JoystickButton(controller, Button.kX.value);
    coneTwoButton.onTrue(movePos2);
    JoystickButton coneThreeButton = new JoystickButton(controller, Button.kY.value);
    coneThreeButton.onTrue(movePos3);


    JoystickButton clawOpenButton = new JoystickButton(controller, Button.kLeftBumper.value);
    clawOpenButton.onTrue(new grab(clawControl, 0));
    JoystickButton clawCubeButton = new JoystickButton(controller, Button.kRightBumper.value);
    clawCubeButton.onTrue(new grab(clawControl, 1));
    JoystickButton clawConeButton = new JoystickButton(controller, Button.kStart.value);
    clawConeButton.onTrue(new grab(clawControl, 2));


    recCommand = new rec(mControl, arm, clawControl, leftShaft, rightShaft, controller);

    /* 
    JoystickButton recButton = new JoystickButton(rightShaft, 11);
    JoystickButton recButton2 = new JoystickButton(rightShaft, 10);
    recButton.onTrue(recCommand.until(recButton2));

    JoystickButton playBack = new JoystickButton(rightShaft, 6);
    playBack.onTrue(playB);  
     */
      
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public void resetArm() {
    arm.resetEncoder();
  }
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    //return new PlaceAndBalance(mControl, arm, clawControl);
    return new newAuto(mControl, arm, clawControl);
  }
}
