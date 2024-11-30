// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

	public static final class DriveConstants {

		public static final double kPZeroLock = 0.001;
		// Driving Parameters - Note that these are not the maximum capable speeds of
		// the robot, rather the allowed maximum speeds
		public static final double kMaxSpeedMetersPerSecond = 4.8; // changed for inspire loudoun
		public static final double kMaxAngularSpeed = 2 * Math.PI * 1; // set the last number back to 1 for normal speed

		public static final double kDirectionSlewRate = 4.3; // radians per second
		public static final double kMagnitudeSlewRate = 4.5; // percent per second (1 = 100%)
		public static final double kRotationalSlewRate = 4.5; // percent per second (1 = 100%)

		public static final double VISION_TURN_kP = 0.01;

		// Chassis configuration
		public static final double kTrackWidth = Units.inchesToMeters(26.5);
		// Distance between centers of right and left wheels on robot
		public static final double kWheelBase = Units.inchesToMeters(26.5);
		// Distance between front and back wheels on robot
		public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
				new Translation2d(kWheelBase / 2, kTrackWidth / 2), new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
				new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
				new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

		// Angular offsets of the modules relative to the chassis in radians
		public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
		public static final double kFrontRightChassisAngularOffset = 0;
		public static final double kBackLeftChassisAngularOffset = Math.PI;
		public static final double kBackRightChassisAngularOffset = Math.PI / 2;

		// SPARK MAX CAN IDs
		public static final int kFrontLeftDrivingCanId = 4;
		public static final int kRearLeftDrivingCanId = 8;
		public static final int kFrontRightDrivingCanId = 2;
		public static final int kRearRightDrivingCanId = 6;

		public static final int kFrontLeftTurningCanId = 3;
		public static final int kRearLeftTurningCanId = 7;
		public static final int kFrontRightTurningCanId = 1;
		public static final int kRearRightTurningCanId = 5;

		public static final int kArmCanId1 = 15;
		public static final int kArmCanId2 = 14;
		public static final int kShooter1CanId = 11;
		public static final int kShooter2CanId = 12;
		public static final int kIntakeIndexCanId = 16;
		public static final int kIntakeMotorCanId = 13;
		public static final int kLaserCanID = 17;

		public static final boolean kGyroReversed = false;
	}

	public static final class VisionConstants {
		// field information
		public static final double ampHeightBottom = Units.feetToMeters(2) + Units.inchesToMeters(2); // in meters
		public static final double pocketHeight = Units.feetToMeters(1) + Units.inchesToMeters(6);
		public static final double ampHeightTop = ampHeightBottom + pocketHeight;
		public static final double AMP_HEIGHT_CENTER = (ampHeightBottom + ampHeightTop) / 2;

		public static final double[] aprilTagHeights = { // These are to the bottom of the AprilTag
				Units.feetToMeters(4) + Units.inchesToMeters(1.0 / 8), // Blue Source
				Units.feetToMeters(4) + Units.inchesToMeters(1.0 / 8), // Blue Source
				Units.feetToMeters(4) + Units.inchesToMeters(3 + 7.0 / 8), // Red Speaker
				Units.feetToMeters(4) + Units.inchesToMeters(3 + 7.0 / 8), // Red Speaker
				Units.feetToMeters(4) + Units.inchesToMeters(1.0 / 8), // Red Amp
				Units.feetToMeters(4) + Units.inchesToMeters(1.0 / 8), // Blue Amp
				Units.feetToMeters(4) + Units.inchesToMeters(3 + 7.0 / 8), // Blue Speaker
				Units.feetToMeters(4) + Units.inchesToMeters(3 + 7.0 / 8), // Blue Speaker
				Units.feetToMeters(4) + Units.inchesToMeters(1.0 / 8), // Blue Source
				Units.feetToMeters(4) + Units.inchesToMeters(1.0 / 8), // Blue Source
				Units.feetToMeters(3) + Units.inchesToMeters(11.5), // Red Stage
				Units.feetToMeters(3) + Units.inchesToMeters(11.5), // Red Stage
				Units.feetToMeters(3) + Units.inchesToMeters(11.5), // Red Stage
				Units.feetToMeters(3) + Units.inchesToMeters(11.5), // Blue Stage
				Units.feetToMeters(3) + Units.inchesToMeters(11.5), // Blue Stage
				Units.feetToMeters(3) + Units.inchesToMeters(11.5), // Blue Stage
		};

		public static final double[] aprilTagUniqueHeights = { // These are to the bottom of the AprilTag
				Units.feetToMeters(4) + Units.inchesToMeters(1.0 / 8), // Source
				Units.feetToMeters(4) + Units.inchesToMeters(3 + 7.0 / 8), // Speaker
				Units.feetToMeters(4) + Units.inchesToMeters(1.0 / 8), // Amp
				Units.feetToMeters(3) + Units.inchesToMeters(11.5), // Stage
		};
		public static final AprilTagFieldLayout kTagLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();

		public static final double speakerHeightBottom = Units.feetToMeters(6) + Units.inchesToMeters(6);
		public static final double speakerHeightTop = Units.feetToMeters(6) + Units.inchesToMeters(10 + 7 / 8);
		public static final double plasticTagLength = Units.inchesToMeters(10.5);
		public static final double aluminumTagLength = Units.inchesToMeters(9);
		public static final double aprilTagLength = Units.inchesToMeters(8 + (1 / 8));

		public static final double X_OFFSET_CAMERA_TO_PIVOT = 1.0; // TODO THIS IS NOT MEASURED YET
		public static final double Y_OFFSET_CAMERA_TO_PIVOT = 1.0; // TODO THIS IS NOT MEASURED YET
		public static final double Z_OFFSET_CAMERA_TO_PIVOT = 1.0; // TODO THIS IS NOT MEASURED YET
		public static final double ARM_LENGTH = 1.0; // TODO ALSO NOT MEASURED
		// Constants such as camera and target height stored. Change per robot and goal!
		public static final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(42);
		public static final double SPEAKER_HEIGHT_METERS = Units.feetToMeters(2.66);
		// Angle between horizontal and the camera.
		public static final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);
		public static final double ARM_ANGLE_HORIZONTAL_OFFSET = Math.toRadians(21);
		// How far from the target we want to be
		public static final double OFFSET_ANGLE_DEGREES = Math.toRadians(40); // radians
		public static final double SPEAKER_TAG_HEIGHT = aprilTagUniqueHeights[1]
				+ plasticTagLength / 2; // meters
		public static final double DISTANCE_TAG_TO_SPEAKER = SPEAKER_HEIGHT_METERS - SPEAKER_TAG_HEIGHT; // meters
		public static final double radians_per_rotation = 2 * Math.PI;
		public static final double rotations_per_radians = 1 / radians_per_rotation;
		public static final double DISTANCE_CAMERA_TO_TAG_Y = SPEAKER_TAG_HEIGHT - CAMERA_HEIGHT_METERS;
	}

	public static final class ModuleConstants {
		// The MAXSwerve module can be configured with one of three pinion gears: 12T,
		// 13T, or 14T.
		// This changes the drive speed of the module (a pinion gear with more teeth
		// will result in a
		// robot that drives faster).
		public static final int kDrivingMotorPinionTeeth = 14;

		// Invert the turning encoder, since the output shaft rotates in the opposite
		// direction of
		// the steering motor in the MAXSwerve Module.
		public static final boolean kTurningEncoderInverted = true;

		// Calculations required for driving motor conversion factors and feed forward
		public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
		public static final double kWheelDiameterMeters = 0.0762;
		public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
		// 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
		// teeth on the bevel pinion
		public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
		public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
				/ kDrivingMotorReduction;

		public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
				/ kDrivingMotorReduction; // meters
		public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
				/ kDrivingMotorReduction) / 60.0; // meters per second

		public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
		public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

		public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
		public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

		public static final double kDrivingP = 0.22;
		public static final double kDrivingI = 0;
		public static final double kDrivingD = 0;
		public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
		public static final double kDrivingMinOutput = -1;
		public static final double kDrivingMaxOutput = 1;

		public static final double kTurningP = 1;
		public static final double kTurningI = 0;
		public static final double kTurningD = 0;
		public static final double kTurningFF = 0;
		public static final double kTurningMinOutput = -1;
		public static final double kTurningMaxOutput = 1;

		public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
		public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

		public static final int kDrivingMotorCurrentLimit = 40; // amps //changed from 50 -> 40 on 4/5
		public static final int kTurningMotorCurrentLimit = 20; // amps

		public static final int kEncoderCPR = 42;

		public static final double kDriveEncoderDistancePerPulse = (kWheelDiameterMeters * Math.PI)
				/ (double) kEncoderCPR;
	}

	public static final class OIConstants {
		public static final int kDriverControllerPort = 0;
		public static final int kOperatorControllerPort = 1;
		public static final double kDriveDeadband = 0.10;
		public static final double kArmDeadband = 0.10;
	}

	public static final class AutoConstants {
		public static final double kMaxSpeedMetersPerSecond = 1;
		public static final double kMaxAccelerationMetersPerSecondSquared = 1;
		public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
		public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

		public static final double kPXController = 4;
		public static final double kPYController = 4;
		public static final double kPThetaController = 4;

		public static final double normalLaserCAN = 0.20;

		// Constraint for the motion profiled robot angle controller
		public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
				kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
	}

	public static final class NeoMotorConstants {
		public static final double kFreeSpeedRpm = 5676;
	}
}
