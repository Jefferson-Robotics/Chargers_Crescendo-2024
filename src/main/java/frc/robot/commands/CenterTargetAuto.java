// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSerial;

public class CenterTargetAuto extends Command {
  /** Creates a new CenterTargetAuto. */
  VisionSerial camera;
  DriveSubsystem swerve;
  private boolean isFinished;
  private int tagX;
  private int tagDistance;
  private double tagRotate;

  private double locationX;
  private double locationY;
  private double controlRotate;

  public CenterTargetAuto(VisionSerial camera, DriveSubsystem swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.camera = camera;
    this.swerve = swerve;
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;

    swerve.drive(0, 0,0,  true, true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    tagX = camera.getCenterX();
    tagDistance = camera.getDistance();

    if (tagX != -1) {
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
          List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
          // End 3 meters straight ahead of where we started, facing forward
          new Pose2d(3, 0, new Rotation2d(0)),
          config);

      var thetaController = new ProfiledPIDController(
          AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
      thetaController.enableContinuousInput(-Math.PI, Math.PI);

      SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
          tagTrajectory,
          swerve::getPose, // Functional interface to feed supplier
          DriveConstants.kDriveKinematics,

          // Position controllers
          new PIDController(AutoConstants.kPXController, 0, 0),
          new PIDController(AutoConstants.kPYController, 0, 0),
          thetaController,
          swerve::setModuleStates,
          swerve);

      // Reset odometry to the starting pose of the trajectory.
      swerve.resetOdometry(tagTrajectory.getInitialPose());

      // Run path following command, then stop at the end.
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.drive(0, 0, 0, false, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
