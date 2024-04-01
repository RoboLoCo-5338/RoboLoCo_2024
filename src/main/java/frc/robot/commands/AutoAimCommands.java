package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class AutoAimCommands {
    public static Command autoAim() {
        return new InstantCommand(
          () -> RobotContainer.m_AutoAim.autoAim(),
          RobotContainer.m_Arm
        );
      }
}