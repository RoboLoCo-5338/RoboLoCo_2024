package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Vision;

public class VisionCommands {
  public static PIDController turnController =
      new PIDController(0, 0, 0); // TODO: Update these values

  public static Command turnToTag() {
    turnController.enableContinuousInput(-180, 180);
    return new PIDCommand(
        turnController,
        () -> RobotContainer.m_robotDrive.getHeading(),
        RobotContainer.m_robotDrive.getHeading() + Vision.getTargetYaw(),
        (rotationalSpeed) ->
            RobotContainer.m_robotDrive.drive(
                0, 0, rotationalSpeed, false, true, true, true), // TODO: Update these values
        RobotContainer.m_robotDrive);
  }
}
