// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.RobotContainer;

public final class DriveCommands {
  public static Command driveForwardTimed(long time, double speed) {
    return new TimedCommand(time, () -> RobotContainer.m_robotDrive.driveSpeed(speed), RobotContainer.m_robotDrive::stop, RobotContainer.m_robotDrive);
  }
}
