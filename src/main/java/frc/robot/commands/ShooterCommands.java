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

/**
 * class for shooter commands
 */
public class ShooterCommands {

  private static long startTime;

  
    /**
     * @return Returns a command to spin the shooter backwards
     */
    public static Command shooterReverse() {
      return new InstantCommand(
        () -> RobotContainer.m_shooter.shooterReverse(),
        RobotContainer.m_shooter
      );
    }

    /**
     * @return Returns a command to spin the shooter forwards
     */
    public static Command shooterForward() {
      return new InstantCommand(
        () -> RobotContainer.m_shooter.shooterForward(),
        RobotContainer.m_shooter
      );

    }

    /**
     * @return Returns a command to stop the shooter
     */
    public static Command shooterStop() {
      return new InstantCommand(
        () -> RobotContainer.m_shooter.shooterStop(),
        RobotContainer.m_shooter
      );
    }

    /**
     * @param time time to spin in milliseconds
     * @return Command to run the shooter forward for a given duration
     */
    public static Command runShooterForwardTimed(long time){
      return new FunctionalCommand(() -> {
        RobotContainer.m_shooter.shooterStop();
        startTime = System.currentTimeMillis();
      },() -> RobotContainer.m_shooter.shooterForward(),interrupted -> RobotContainer.m_shooter.shooterStop(), () -> System.currentTimeMillis()-time>startTime, RobotContainer.m_shooter);
    }

}