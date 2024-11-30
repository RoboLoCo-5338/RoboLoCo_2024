// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Vision;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
	private Command m_autonomousCommand;

	private RobotContainer m_robotContainer;

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		// Instantiate our RobotContainer. This will perform all our button bindings,
		// and put our
		// autonomous chooser on the dashboard.
		m_robotContainer = new RobotContainer();

		// CameraServer.startAutomaticCapture();
		// Run a warmup to to warm up the relevant libraries
		FollowPathCommand.warmupCommand().schedule();
	}

	/**
	 * This function is called every 20 ms, no matter the mode. Use this for items
	 * like diagnostics that you want ran during disabled, autonomous, teleoperated
	 * and test.
	 *
	 * <p>
	 * This runs after the mode specific periodic functions, but before LiveWindow
	 * and SmartDashboard integrated updating.
	 */
	@Override
	public void robotPeriodic() {
		// Runs the Scheduler. This is responsible for polling buttons, adding
		// newly-scheduled
		// commands, running already-scheduled commands, removing finished or
		// interrupted commands,
		// and running subsystem periodic() methods. This must be called from the
		// robot's periodic
		// block in order for anything in the Command-based framework to work.
		SmartDashboard.putBoolean("LaserCan is Note", RobotContainer.m_Intake.isNote());

		// SmartDashboard.putNumber("Heading",
		// RobotContainer.m_robotDrive.getHeading());
		// SmartDashboard.putNumber("X Pose",
		// RobotContainer.m_robotDrive.getPose().getX());
		// SmartDashboard.putNumber("Y Pose",
		// RobotContainer.m_robotDrive.getPose().getY());
		// SmartDashboard.putNumber("Angle",
		// RobotContainer.m_robotDrive.m_gyro.getAngle());
		// SmartDashboard.putNumber("Front left speed",
		// RobotContainer.m_robotDrive.m_frontLeft.getState().speedMetersPerSecond);
		// SmartDashboard.putNumber("Front right speed",
		// RobotContainer.m_robotDrive.m_frontRight.getState().speedMetersPerSecond);
		// SmartDashboard.putNumber("Rear left speed",
		// RobotContainer.m_robotDrive.m_rearLeft.getState().speedMetersPerSecond);
		// SmartDashboard.putNumber("Rear right speed",
		// RobotContainer.m_robotDrive.m_rearRight.getState().speedMetersPerSecond);

		// SmartDashboard.putNumber("Front left distance (meters)",
		// RobotContainer.m_robotDrive.m_frontLeft.getPosition().distanceMeters);
		// SmartDashboard.putNumber("Front right distance (meters)",
		// RobotContainer.m_robotDrive.m_frontRight.getPosition().distanceMeters);
		// SmartDashboard.putNumber("Rear left distance (meters)",
		// RobotContainer.m_robotDrive.m_rearLeft.getPosition().distanceMeters);
		// SmartDashboard.putNumber("Rear right distance (meters)",
		// RobotContainer.m_robotDrive.m_rearRight.getPosition().distanceMeters);

		SmartDashboard.putNumber("Arm Encoder Value", RobotContainer.m_Arm.getArmPosition());
		SmartDashboard.putNumber("Vision Yaw", Vision.getTargetYaw());

		CommandScheduler.getInstance().run();
	}

	/** This function is called once each time the robot enters Disabled mode. */
	@Override
	public void disabledInit() {
	}

	@Override
	public void disabledPeriodic() {
	}

	/**
	 * This autonomous runs the autonomous command selected by your
	 * {@link RobotContainer} class.
	 */
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
		SmartDashboard.putBoolean("Laser can distance", RobotContainer.m_Intake.isNote());
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

		RobotContainer.m_Arm.resetArm();
		RobotContainer.m_Arm.stopArm();
		RobotContainer.m_robotDrive.drive(0, 0, 0, false, false, false, false); // added 6/1/24 to straighten wheels
																				// upon teleop
		// init for testing

	}

	/** This function is called periodically during operator control. */
	@Override
	public void teleopPeriodic() {
		// Calculate drivetrain commands from Joystick values
		double forward = -RobotContainer.m_driverController.getLeftY()
				* Constants.DriveConstants.kMaxSpeedMetersPerSecond;
		double strafe = -RobotContainer.m_driverController.getLeftX()
				* Constants.DriveConstants.kMaxSpeedMetersPerSecond;
		double turn = -RobotContainer.m_driverController.getRightX() * Constants.DriveConstants.kMaxAngularSpeed;

		// Read in relevant data from the Camera
		boolean targetVisible = Vision.hasResults();
		double targetYaw = 0.0;

		// PhotonPipelineResult result = RobotContainer.camera.getLatestResult();
		// if (result != null) {
		// // Camera processed a new frame since last
		// // Get the last one in the list.
		// if (result.hasTargets()) {
		// // At least one AprilTag was seen by the camera
		// for (var target : result.getTargets()) {
		// if (target.getFiducialId() == 7) {
		// // Found Tag 7, record its information
		// targetYaw = target.getYaw();
		// targetVisible = true;
		// }
		// }
		// }
		// }

		// Auto-align when requested
		if (RobotContainer.m_driverController.a().getAsBoolean() && targetVisible) {
			// Driver wants auto-alignment to tag 7
			// And, tag 7 is in sight, so we can turn toward it.
			// Override the driver's turn command with an automatic one that turns toward
			// the tag.
			turn = -1.0 * targetYaw * Constants.DriveConstants.VISION_TURN_kP
					* Constants.DriveConstants.kMaxAngularSpeed;
		}

		// Command drivetrain motors based on target speeds
		// DriveSubsystem.drive(strafe, forward, turn, true, true, false, false);
		// RobotContainer.m_robotDrive.setDefaultCommand(
		// The left stick controls translation of the robot.
		// Turning is controlled by the X axis of the right stick.
		// new RunCommand(
		// () -> RobotContainer.m_robotDrive.drive(strafe,forward,turn,
		// true, true, false,false),
		// RobotContainer.m_robotDrive));

		// Put debug information to the dashboard
		SmartDashboard.putBoolean("Vision Target Visible", targetVisible);
	}

	@Override
	public void testInit() {
		// Cancels all running commands at the start of test mode.
		CommandScheduler.getInstance().cancelAll();
	}

	/** This function is called periodically during test mode. */
	@Override
	public void testPeriodic() {
	}
}
