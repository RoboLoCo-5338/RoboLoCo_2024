package frc.robot.commands;


import java.sql.Driver;

import org.ejml.equation.Function;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
//shresta made up errors
public class ShooterCommands {

  public static long startTime;

    public static Command shooterReverse() {
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

    public static Command runShooterForwardTimed(double d){
       //new FunctionalCommand()
      return new FunctionalCommand(() -> {
        RobotContainer.m_shooter.shooterStop();
        startTime = System.currentTimeMillis();
      },() -> RobotContainer.m_shooter.shooterForward(),interrupted -> RobotContainer.m_shooter.shooterStop(), () -> System.currentTimeMillis()-d>startTime, RobotContainer.m_shooter);
    }
}