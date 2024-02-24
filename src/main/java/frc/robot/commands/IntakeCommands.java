package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class IntakeCommands {
    
    public static Command arm() {
      return new InstantCommand(
        () -> RobotContainer.m_shooter.shooterReverse(),
        RobotContainer.m_shooter
      );
    }

    public static Command shooterForward() {
      return new InstantCommand(
        () -> RobotContainer.m_shooter.shooterForward(),
        RobotContainer.m_shooter
      );

    }

    public static Command shooterStop() {
      return new InstantCommand(
        () -> RobotContainer.m_shooter.shooterStop(),
        RobotContainer.m_shooter
      );
    }
}
