package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.RobotContainer;

public final class ShooterCommands {
  public static Command shooterReverse() {
    return new InstantCommand(RobotContainer.m_shooter::shooterReverse, RobotContainer.m_shooter);
  }

  public static Command shooterReverseSlow() {
    return new InstantCommand(RobotContainer.m_shooter::shooterReverseSlow, RobotContainer.m_shooter);
  }

  public static Command shooterForward() {
    return new InstantCommand(RobotContainer.m_shooter::shooterForward, RobotContainer.m_shooter);
  }

  public static Command shooterStop() {
    return new InstantCommand(RobotContainer.m_shooter::shooterStop, RobotContainer.m_shooter);
  }

  public static Command runShooterForwardTimed(long time) {
    return new TimedCommand(time, RobotContainer.m_shooter::shooterForward, RobotContainer.m_shooter::shooterStop, RobotContainer.m_shooter);
  }
}
