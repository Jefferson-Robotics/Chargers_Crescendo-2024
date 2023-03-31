// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Xbox;
import frc.robot.commands.cancelAll;
import frc.robot.commands.dockArmEncoder;
import frc.robot.commands.grab;
import frc.robot.commands.joystickControl;
import frc.robot.commands.moveEncodeThird;
import frc.robot.commands.moveEncoder;
import frc.robot.commands.newAuto;
import frc.robot.commands.newBalance;
//import frc.robot.commands.turnNinty;
import frc.robot.commands.rec;
import frc.robot.commands.resetArmEncoders;
import frc.robot.commands.xboxArm;
import frc.robot.subsystems.CANMotorControl;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.armSystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private ShuffleboardTab tab = Shuffleboard.getTab("Game Display");
  private Joystick leftShaft = new Joystick(0);
  private Joystick rightShaft = new Joystick(1);
  private XboxController controller = new XboxController(2);
  private CANMotorControl mControl = new CANMotorControl(tab);
  private armSystem arm = new armSystem(tab);
  private Claw clawControl = new Claw(tab);
  private rec recCommand;
  //private playBack playB = new playBack(mControl, arm, clawControl, tab);
  
  private UsbCamera camera = CameraServer.startAutomaticCapture("Camera", 0);
  //Camera access with a search.
  //private OpenMV camera = new OpenMV();

  //private Reader reader = new Reader(m_motorcontrol);
  private joystickControl joystick = new joystickControl(mControl, leftShaft, rightShaft);
  private Xbox xbox = new Xbox(mControl, controller);
  private xboxArm armControl = new xboxArm(arm, controller, clawControl);
  private SendableChooser<Command> m_Chooser = new SendableChooser<Command>();

  private resetArmEncoders reset = new resetArmEncoders(arm);
  private dockArmEncoder armZero = new dockArmEncoder(arm);
  private moveEncoder movePos1 = new moveEncoder(arm, Constants.bottomFrontStop, -100);
  private moveEncoder movePos2 = new moveEncoder(arm, Constants.bottomFrontStop, -260);
  private moveEncodeThird movePos3 = new moveEncodeThird(arm, Constants.bottomBackStop, Constants.topBackStop);
  private grab open = new grab(clawControl, 0);
  private grab cube = new grab(clawControl, 1);
  private grab cone = new grab(clawControl, 2);
  private newBalance balance = new newBalance(mControl);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //m_motorcontrol.setDefaultCommand(new Reader(m_motorcontrol)/*new Drive(m_motorcontrol, controller)*/);
    /*
     * m_Chooser.setDefaultOption("Playback", playB);
    m_Chooser.addOption("Balance", balance);
    m_Chooser.addOption("Playback with Balance", autoPB);
    tab.add("Autonomous", m_Chooser);
     */
    camera.setResolution(320, 240);
    camera.setFPS(5);
    


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

    JoystickButton resetArmEncoders1 = new JoystickButton(rightShaft, 10);
    resetArmEncoders1.onTrue(reset);
    JoystickButton resetArmEncoders2 = new JoystickButton(rightShaft, 11);
    resetArmEncoders2.onTrue(reset);
    JoystickButton balanceButton = new JoystickButton(leftShaft, 2);
    balanceButton.onTrue(balance);

    /* 
    JoystickButton joyOpen = new JoystickButton(leftShaft, 1);
    joyOpen.onTrue(open);
    JoystickButton joyCube = new JoystickButton(rightShaft, 1);
    joyCube.onTrue(cube);
    JoystickButton joyCone = new JoystickButton(rightShaft, 3);
    joyCone.onTrue(cone);
    */



    JoystickButton dockArmButton = new JoystickButton(controller, Button.kB.value);
    dockArmButton.onTrue(armZero);
    JoystickButton frontOneButton = new JoystickButton(controller, Button.kA.value);
    frontOneButton.onTrue(movePos1);
    JoystickButton coneTwoButton = new JoystickButton(controller, Button.kX.value);
    coneTwoButton.onTrue(movePos2);
    JoystickButton coneThreeButton = new JoystickButton(controller, Button.kY.value);
    coneThreeButton.onTrue(movePos3);

    JoystickButton clawOpenButton = new JoystickButton(controller, Button.kLeftBumper.value);
    clawOpenButton.onTrue(open);
    JoystickButton clawCubeButton = new JoystickButton(controller, Button.kRightBumper.value);
    clawCubeButton.onTrue(cube);
    JoystickButton clawConeButton = new JoystickButton(controller, Button.kStart.value);
    clawConeButton.onTrue(cone);


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
    //return new moveDistance(mControl, 10, 0.6);
  }
}
