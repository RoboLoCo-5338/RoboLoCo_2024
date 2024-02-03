package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.RobotContainer;

public class ShooterCommands {
  
    public static Command shooterReverse() {
      return new InstantCommand(
        () -> RobotContainer.m_shooter.shooterReverse(),
        RobotContainer.m_shooter
      );
    }
    
    public static Command shooterForward() {
      SmartDashboard.putString("help", "please");
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
