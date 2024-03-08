// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.RecordPlaybackConstants;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.AutoPickup;
import frc.robot.commands.CancelAll;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.LiftScissorLift;
import frc.robot.commands.PrimeShooter;
import frc.robot.commands.ScissorIntake;
import frc.robot.commands.ScissorOuttake;
import frc.robot.commands.ShootNote;
import frc.robot.commands.playBack;
import frc.robot.commands.rec;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.NoteActuator;
import frc.robot.subsystems.Onboarder;
import frc.robot.subsystems.Shooter;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final Climb climb = new Climb();
  private final Onboarder onboarder = new Onboarder();
  private final Shooter shooter = new Shooter();
  private final NoteActuator noteActuator = new NoteActuator();
  private final CancelAll cancelAll = new CancelAll(onboarder, climb, m_robotDrive, shooter, noteActuator);

  //private final VisionSerial visionTag = new VisionSerial();
  //private final Camera camera = new Camera();
  // The driver's controller
  XboxController m_driverController = new XboxController(OperatorConstants.kDriverControllerPort);
  private Joystick leftShaft = new Joystick(OperatorConstants.kDriverJoystickLeft);
  private Joystick rightShaft = new Joystick(OperatorConstants.kDriverJoystickRight);
  //CommandXboxController commandController = new CommandXboxController(OIConstants.kDriverControllerPort);
  //private final AutoPickup autoPickup = new AutoPickup(m_robotDrive, onboarder, camera);

  // Robot Mechanisms
  private final PrimeShooter primeShooter = new PrimeShooter(shooter);
  private final ShootNote shootNote = new ShootNote(shooter, onboarder);
  private final ClimbCommand climbCommand = new ClimbCommand(climb, m_driverController);
  private final AutoIntake autoIntake = new AutoIntake(onboarder);

  // Shuffleboard
  private final ShuffleboardTab tab = Shuffleboard.getTab("Autonomous");
  private final GenericEntry alliancebox = tab.add("Red Alliance", false).withWidget(BuiltInWidgets.kToggleButton).getEntry();
  private final SendableChooser<File> fileChooser = new SendableChooser<>();
  private final GenericEntry fileName = tab.add("File Name", "PLACEHOLDER")
   .withWidget(BuiltInWidgets.kTextView).getEntry();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  private rec recordCommand;
  private playBack playbackCommand;
  private Boolean onRedAlliance = true;
  //private IRBeamBreaker intakeSensor = new IRBeamBreaker(8);

  public RobotContainer() {
    File[] files = RecordPlaybackConstants.kRecordDirectory.listFiles();
    for (int i = 0; i < files.length; i++) {
      fileChooser.addOption(files[i].getName(), files[i]);
    }
    tab.add("Autonomous Mode", fileChooser)
    .withWidget(BuiltInWidgets.kComboBoxChooser)
    .withPosition(0, 0)
    .withSize(2, 1);

    // Configure the trigger bindings
    configureBindings();

    // Configure default commands
    /* 
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
            */
    climb.setDefaultCommand(climbCommand);
    m_robotDrive.setDefaultCommand(new DriveWithJoysticks(m_robotDrive, leftShaft, rightShaft));

    // SECONDARY DRIVER BUTTONS
    noteActuator.setDefaultCommand(
      new RunCommand(
        ()->{
          if(m_driverController.getLeftTriggerAxis() > .1){
            noteActuator.setRoller(m_driverController.getLeftTriggerAxis());
          } else if(m_driverController.getRightTriggerAxis() > .1) {
            noteActuator.setRoller(-m_driverController.getRightTriggerAxis());
          } else {
            noteActuator.setRoller(0);
          }
        }
        , onboarder)
    );
    shooter.setDefaultCommand(
      new RunCommand(
        ()->{
          if(m_driverController.getBackButton()){
            shooter.shoot(-1);
          } else if (m_driverController.getStartButton()) {
            shooter.shoot(1);
          } else {
            shooter.shoot(0);
          }
        }, shooter)
    );
    noteActuator.setDefaultCommand(new LiftScissorLift(noteActuator, m_driverController));
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
    
    recordCommand = new rec(m_robotDrive, onboarder, shooter, noteActuator, leftShaft, rightShaft, fileChooser, fileName);
    
    // RECORD AND PLAYBACK
    JoystickButton recButton = new JoystickButton(rightShaft, 10);
    JoystickButton recButton2 = new JoystickButton(rightShaft, 11);
    recButton.onTrue(recordCommand.until(recButton2));

    playbackCommand = new playBack(m_robotDrive, onboarder, shooter, noteActuator, fileChooser, alliancebox);
    JoystickButton playBack = new JoystickButton(rightShaft, 7);
    playBack.onTrue(playbackCommand);

    JoystickButton cancleAllSecondary = new JoystickButton(m_driverController, 5);
    cancleAllSecondary.onTrue(cancelAll);
    // JOYSTICK BUTTON BINDINGS
    JoystickButton primeShooterButton = new JoystickButton(m_driverController, Button.kA.value);
    JoystickButton shootButton = new JoystickButton(m_driverController, Button.kX.value);
    primeShooterButton.onTrue(primeShooter);
    shootButton.onTrue(shootNote);

    JoystickButton intakeSource = new JoystickButton(m_driverController, Button.kB.value);
    JoystickButton outtakeAmp = new JoystickButton(m_driverController, Button.kY.value);
    intakeSource.onTrue(new ScissorIntake(noteActuator));
    outtakeAmp.onTrue(new ScissorOuttake(noteActuator));

    new JoystickButton(leftShaft, 10)
        .whileTrue(new RunCommand(
            () -> {
              m_robotDrive.resetGyro();
        }));
    new JoystickButton(leftShaft, 11)
        .whileTrue(new RunCommand(
            () -> {
              m_robotDrive.resetGyro();
        }));
    
    new JoystickButton(rightShaft, 3)
    .whileTrue(new RunCommand(
        () -> {
        shooter.shoot(1);
    }));

    new JoystickButton(leftShaft, 4)
    .whileTrue(new RunCommand(
        () -> {
          onboarder.setSpeed(1);
    }))
    .whileFalse(new RunCommand(
        () -> {
          onboarder.setSpeed(0);
    }));

    new JoystickButton(leftShaft, 5)
    .whileTrue(new RunCommand(
        () -> {
          onboarder.setSpeed(-1);
    }))
    .whileFalse(new RunCommand(
        () -> {
          onboarder.setSpeed(0);
    }));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
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
        new Pose2d(3, 0, Rotation2d.fromRadians(0)),
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
  }
}
