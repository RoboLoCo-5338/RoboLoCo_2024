package frc.robot.commands;


import java.sql.Driver;

import org.ejml.equation.Function;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;

public class ShooterCommands {

  private static long startTime;

  
    public static Command shooterReverse() {
      return new InstantCommand(
        () -> RobotContainer.m_shooter.shooterReverse(),
        RobotContainer.m_shooter
      );
    }

      public static Command shooterReverseSlow() {
      return new InstantCommand(
        () -> RobotContainer.m_shooter.shooterReverseSlow(),
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

    public static Command runShooterForwardTimed(long time){
      return new FunctionalCommand(() -> {
        RobotContainer.m_shooter.shooterStop();
        startTime = System.currentTimeMillis();
      },() -> RobotContainer.m_shooter.shooterForward(),interrupted -> RobotContainer.m_shooter.shooterStop(), () -> System.currentTimeMillis()-time>startTime, RobotContainer.m_shooter);
    }

}