package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.OIConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Vision;

public class VisionCommands {
  public static PIDController turnController =
      new PIDController(0.1, 0, 0); // TODO: Update these values

  public static Command turnToTarget() {
    turnController.enableContinuousInput(-180, 180);
    
    return new PIDCommand(
        turnController,
        () -> RobotContainer.m_robotDrive.getHeading(),
        // RobotContainer.m_robotDrive.getHeading() - Vision.getTargetYaw() ,
        () -> {
            if(Vision.getTargetYaw() == Double.POSITIVE_INFINITY)
                return RobotContainer.m_robotDrive.getHeading();
            return RobotContainer.m_robotDrive.getHeading() - Vision.getTargetYaw();
        },
        (rotationalSpeed) ->
            RobotContainer.m_robotDrive.drive(
                -MathUtil.applyDeadband(
                    RobotContainer.m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(
                    RobotContainer.m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                rotationalSpeed,
                true,
                true,
                true,
                false), // TODO: Update these values
        RobotContainer.m_robotDrive);
  }
}
