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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AprilTagConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSerial;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoSourceAlign extends SequentialCommandGroup {

  /** Creates a new AutoSourceAlign. */
  public AutoSourceAlign(VisionSerial camera, DriveSubsystem swerve) {
    // Add your commands in the addCommands() call, e.g.
    double tagDistance = camera.getDistance(1);
    double angle = -camera.getAngle(1);

    double robotAngle = (Math.PI * (swerve.getAngle() % 360)) / 180;
    double tagRotation = (AprilTagConstants.tagAngles[1] - robotAngle);

    double translateX = 0.01;
    double translateY = 0.01;
    double controlRotate = 0.01;

    if (tagDistance != -1) {
      tagDistance = tagDistance / 100; // cm to m

      //translateX = tagDistance * Math.cos(angle) - 1.5 * Math.sin(tagRotation);
      //translateX -= 0.5;
      //translateY = tagDistance * Math.sin(angle) - 1.5 * Math.cos(tagRotation);

      //controlRotate = tagRotation;
    }

    controlRotate = Math.PI;
    translateX = 0;
    translateY = 0;

    System.out.println("TransX: " + translateX + " | TransY: " + translateY + " | Rotate: " + ((controlRotate / Math.PI) * 180));

    TrajectoryConfig config = new TrajectoryConfig(
        1.5 /*AutoConstants.kMaxSpeedMetersPerSecond*/,
        1.5 /*AutoConstants.kMaxAccelerationMetersPerSecondSquared*/)

        .setKinematics(DriveConstants.kDriveKinematics);

    Trajectory tagTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0,0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(),
        new Pose2d(1,0, Rotation2d.fromRadians(((swerve.getHeading() * Math.PI) / 180) + Math.PI)),
        config
        );

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // Reset odometry to the starting pose of the trajectory.
    swerve.resetOdometry(tagTrajectory.getInitialPose());

    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new SwerveControllerCommand(
        tagTrajectory,
        swerve::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        swerve::setModuleStates,
        swerve).andThen(() -> swerve.drive(0, 0, 0, false, false)));
  }
}
