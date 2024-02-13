// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;


import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
import frc.robot.Constants.DriveConstants;
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private double rotationSpeed;
  private RobotContainer m_robotContainer;

  // Constants such as camera and target height stored. Change per robot and goal!
  final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(42);
  final double TARGET_HEIGHT_METERS = Units.feetToMeters(2.66);
  // Angle between horizontal and the camera.
  final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);
  // How far from the target we want to be
  final double GOAL_RANGE_METERS = Units.feetToMeters(3);

  // Change this to match the name of your camera
  static PhotonCamera camera = new PhotonCamera("Captain_Rivets");
  // final double LINEAR_P = 0.001;
  // final double LINEAR_D = 0.0;
  // PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);
  final double ANGULAR_P = 0.01;
  final double ANGULAR_D = 0.0;
  PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

  // Need 3d photon vision and wpilib 2024 so that we can get field layout
  AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.
  PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera, robotToCam);
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
    }


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    DriveConstants.armHashMap = new HashMap<>(70);
    
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("Pitch", RobotContainer.navX.getPitch());
    SmartDashboard.putNumber("Roll", RobotContainer.navX.getRoll());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // SmartDashboard.putString("Dino Rivets ftw","HELLO");
    
    // SmartDashboard.putNumber("Elevator Position Periodic", RobotContainer.m_Elevator.getElevatorPosition());
    // SmartDashboard.putNumber("Arm Position", RobotContainer.m_Arm.getArmPosition());
    // SmartDashboard.putNumber("effector Encoder position", RobotContainer.effector.armAbsEncoder.getPosition());
    SmartDashboard.putBoolean("direction", (RobotContainer.reverseModifier > 0));

    // TODO fix: forwardSpeed = -RobotContainer.controller1.getRawAxis(3);
    SmartDashboard.putString("Test Before Test", "Working tho");
    
    // TODO fix: RobotContainer.limeLight.whileTrue(turnToTarg());
    // Use our forward/turn speeds to control the drivetrain
  }

  public Command turnToTarg() {
    return new RunCommand(
      () -> turnToTarg2()
    );
  }

  public void turnToTarg2() {
    // Vision-alignment mode
    // Query the latest result from PhotonVision
    PhotonPipelineResult result = camera.getLatestResult();
    PhotonTrackedTarget best_result = null;
    if (result.hasTargets()) {
      best_result = result.getBestTarget();
      double range = 
              PhotonUtils.calculateDistanceToTargetMeters(
                      CAMERA_HEIGHT_METERS, 
                      TARGET_HEIGHT_METERS, 
                      CAMERA_PITCH_RADIANS, 
                      Units.degreesToRadians(best_result.getPitch()));
      // TODO fix: forwardSpeed = -RobotContainer.controller1.getY();
      // Calculate angular turn power
      // -1.0 required to ensure positive PID controller effort _increases_ yaw
      rotationSpeed = -turnController.calculate(best_result.getYaw(), 0);
      SmartDashboard.putNumber("Distance from April Tag", range);
    } else {
      rotationSpeed = 0;
    }
    // if making a max speed
    if (rotationSpeed > 0.1) {
      rotationSpeed = 0.1;
    } 
    if (rotationSpeed < -0.1) {
      rotationSpeed = -0.1;
    }
    // turns based on yaw
    if (best_result != null && result.hasTargets() && best_result.getYaw() < 1) {
      SmartDashboard.putString("RotationSpeed", Double.toString(rotationSpeed));
      // TODO Move using swerve
      // RobotContainer.drivetrain.tankDrive(rotationSpeed - Constants.min_command, -rotationSpeed + Constants.min_command);
    } else if (best_result != null && result.hasTargets() && best_result.getYaw() > 1) {
      SmartDashboard.putString("-RotationSpeed", Double.toString(rotationSpeed));
      // TODO Move using swerve
      // RobotContainer.drivetrain.tankDrive(rotationSpeed + Constants.min_command, -rotationSpeed - Constants.min_command);
    }
    }

    // if (result.hasTargets()) {
    //     // Calculate angular turn power
    //     // -1.0 required to ensure positive PID controller effort _increases_ yaw
    //     rotationSpeed = -result.getBestTarget().getYaw();
    //     double range = PhotonUtils.calculateDistanceToTargetMeters(0, 0.1397, CAMERA_PITCH_RADIANS, Units.degreesToRadians(result.getBestTarget().getPitch()));
    //     SmartDashboard.putString("Distance", Double.toString(range));
    //     if (range < 1) {
    //       if (Math.abs(rotationSpeed) > 1.0) {
    //         if (rotationSpeed < 0) {
    //           RobotContainer.drivetrain.tankDrive((Constants.kp * (forwardSpeed - rotationSpeed) + Constants.min_command), (Constants.kp * (forwardSpeed + rotationSpeed) - Constants.min_command));
    //         } else {
    //           RobotContainer.drivetrain.tankDrive((Constants.kp * (forwardSpeed - rotationSpeed) - Constants.min_command), (Constants.kp * (forwardSpeed + rotationSpeed) + Constants.min_command));
    //         }
    //       } else {
    //       // If we have no targets, tstay still.
    //       rotationSpeed = 0;
    //       }
    //     } else {
    //       if (Math.abs(rotationSpeed) > 1.0) {
    //         if (rotationSpeed < 0) {
    //           RobotContainer.drivetrain.tankDrive((Constants.kp * (forwardSpeed - rotationSpeed) + Constants.min_command)/range, (Constants.kp * (forwardSpeed + rotationSpeed) - Constants.min_command)/range);
    //         } else {
    //           RobotContainer.drivetrain.tankDrive((Constants.kp * (forwardSpeed - rotationSpeed) - Constants.min_command)/range, (Constants.kp * (forwardSpeed + rotationSpeed) + Constants.min_command)/range);
    //         }
    //       } else {
    //       // If we have no targets, tstay still.
    //       rotationSpeed = 0;
    //       }
    //       }
    // } 



  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}

