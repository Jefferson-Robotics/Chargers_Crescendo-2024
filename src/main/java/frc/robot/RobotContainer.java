// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutoRotate;
import frc.robot.commands.AutoSourceAlign;
import frc.robot.commands.playBack;
import frc.robot.commands.rec;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSerial;

import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private ShuffleboardTab tab = Shuffleboard.getTab("Record and Playback");
  private String recFileName = "swerveRecord";
  private Integer fileID = 1;
  private VisionSerial vision = new VisionSerial();
  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  private rec recordCommand;
  private playBack playB = new playBack(m_robotDrive, m_driverController, tab, recFileName, fileID);
  //private CenterOnTarget cameraTrackRotate = new CenterOnTarget(vision, m_robotDrive);

  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY() * -.5, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX() * -.5, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX() * -.5, OIConstants.kDriveDeadband),
                true, true),
            m_robotDrive));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    new JoystickButton(m_driverController, 6)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
    recordCommand = new rec(m_robotDrive, m_driverController, recFileName, fileID);

    
    JoystickButton recButton = new JoystickButton(m_driverController, Button.kA.value);
    JoystickButton recButton2 = new JoystickButton(m_driverController, Button.kB.value);
    recButton.onTrue(recordCommand.until(recButton2));

    JoystickButton playBack = new JoystickButton(m_driverController, Button.kY.value);
    playBack.onTrue(playB);

    new JoystickButton(m_driverController, Button.kBack.value)
        .onTrue(new AutoRotate(vision, m_robotDrive, 90, true));

    new JoystickButton(m_driverController, Button.kStart.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.resetGyro()));

    /*
    JoystickButton cameraTrack = new JoystickButton(m_driverController, Button.kBack.value);
    cameraTrack.onTrue(cameraTrackRotate);
    */
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    /*
    // An example command will be run in autonomous
    double tagDistance = vision.getDistance(1);
    double angle = -vision.getAngle(1);
    double tagRotation = vision.getTagRotation(1);

    double translateX = 0.01;
    double translateY = 0.01;
    double controlRotate = 0.01;

    if (tagDistance != -1) {
      tagDistance = tagDistance / 100; // cm to m

      translateX = tagDistance * Math.cos(angle) - .7 * Math.sin(-(tagRotation + 3 * Math.PI / 2) % (2 * Math.PI));
      //translateX -= 0.5;
      translateY = tagDistance * Math.sin(angle) - .7 * Math.cos(-(tagRotation + 3 * Math.PI / 2) % (2 * Math.PI));

      controlRotate = tagRotation;
    }
    System.out.println("TransX: " + translateX + " | TransY: " + translateY + " | Rotate: " + ((controlRotate / Math.PI) * 180));

    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);
    // An example trajectory to follow. All units in meters.

    Trajectory tagTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(),
        new Pose2d(translateX, translateY, Rotation2d.fromRadians(controlRotate)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        tagTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(tagTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));

    */
    return new AutoSourceAlign(vision, m_robotDrive);
  }
}
