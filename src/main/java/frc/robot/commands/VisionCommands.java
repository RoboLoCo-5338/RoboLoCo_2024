package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Vision;

public class VisionCommands {
	public static PIDController turnController = new PIDController(0.1, 0, 0); // TODO: Update these values

	public static Command turnToTarget() {
		turnController.enableContinuousInput(-180, 180);

		return new PIDCommand(turnController, () -> RobotContainer.m_robotDrive.getHeading(),
				// RobotContainer.m_robotDrive.getHeading() - Vision.getTargetYaw() ,
				() -> {
					if (Vision.getTargetYaw() == Double.POSITIVE_INFINITY)
						return RobotContainer.m_robotDrive.getHeading();
					return RobotContainer.m_robotDrive.getHeading() - Vision.getTargetYaw();
				},
				(rotationalSpeed) -> RobotContainer.m_robotDrive.drive(
						-MathUtil.applyDeadband(RobotContainer.m_driverController.getLeftY(),
								OIConstants.kDriveDeadband),
						-MathUtil.applyDeadband(RobotContainer.m_driverController.getLeftX(),
								OIConstants.kDriveDeadband),
						rotationalSpeed, true, true, true, false), // TODO: Update these values
				RobotContainer.m_robotDrive);
	}

	public static Command turnToTarget(int tagId) {
		turnController.enableContinuousInput(-180, 180);

		return new PIDCommand(turnController, () -> RobotContainer.m_robotDrive.getHeading(),
				// RobotContainer.m_robotDrive.getHeading() - Vision.getTargetYaw() ,
				() -> {
					if (Vision.getTargetYaw() == Double.POSITIVE_INFINITY)
						return RobotContainer.m_robotDrive.getHeading();
					return RobotContainer.m_robotDrive.getHeading() - Vision.getTargetYaw(tagId);
				},
				(rotationalSpeed) -> RobotContainer.m_robotDrive.drive(
						-MathUtil.applyDeadband(RobotContainer.m_driverController.getLeftY(),
								OIConstants.kDriveDeadband),
						-MathUtil.applyDeadband(RobotContainer.m_driverController.getLeftX(),
								OIConstants.kDriveDeadband),
						rotationalSpeed, true, true, true, false), // TODO: Update these values
				RobotContainer.m_robotDrive);
	}

	public static Command autoAim() {
		int tag = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue ? 7 : 4;
		if (Vision.getTagTarget(tag).isPresent())
			return turnToTarget(tag)
					.andThen(ArmCommands.setArmRadians(getOptimalAngleRadians(Vision.getDistanceToTarget(tag),
							Vision.getTargetYaw(tag), getCurrentAngle())));
		return Commands.none();
	}

	public static double getOptimalAngleRadians(double distanceToTag, double angleToTag, double angleCurrent) {
		double resultDegree;
		double numerator = VisionConstants.DISTANCE_CAMERA_TO_TAG_Y - (VisionConstants.ARM_LENGTH * Math.sin(angleCurrent))
				+ VisionConstants.DISTANCE_TAG_TO_SPEAKER;
		double xDistance = distanceToTag * Math.cos(angleToTag);
		double denominator = xDistance + VisionConstants.X_OFFSET_CAMERA_TO_PIVOT + (VisionConstants.ARM_LENGTH * angleCurrent);
		double fraction = numerator / denominator;
		double offsetRadians = VisionConstants.OFFSET_ANGLE_DEGREES;
		resultDegree = offsetRadians - Math.atan(fraction);
		return resultDegree;
	}

	// public static double getOptimalAngleDegrees(double distanceToTag, double
	// angleToTag, double angleCurrent) {
	// double resultDegree;
	// double numerator = VisionConstants.DISTANCE_CAMERA_TO_TAG_Y
	// - (Constants.ARM_LENGTH * Math.sin(Math.toRadians(angleCurrent)))
	// + VisionConstants.DISTANCE_TAG_TO_SPEAKER;
	// double xDistance = distanceToTag * Math.cos(Math.toRadians(angleToTag));
	// double denominator = xDistance + Constants.X_OFFSET_CAMERA_TO_PIVOT
	// + (Constants.ARM_LENGTH * Math.cos(Math.toRadians(angleCurrent)));
	// double fraction = numerator / denominator;
	// double offsetRadians = VisionConstants.OFFSET_ANGLE_DEGREES;
	// resultDegree = offsetRadians - Math.atan(fraction);
	// return Math.toDegrees(resultDegree);
	// }

	public static double getCurrentAngle() {
		double rotations = RobotContainer.m_Arm.getArmPosition(); // only gets motor 1 but it doesnt matter
		double angle_bad_offset = rotations * VisionConstants.radians_per_rotation;
		double angle_offset_good = angle_bad_offset + VisionConstants.ARM_ANGLE_HORIZONTAL_OFFSET;
		return angle_offset_good;
	}
}
