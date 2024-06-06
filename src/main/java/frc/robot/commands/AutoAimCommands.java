package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.AutoAimSubsystem;

public final class AutoAimCommands {
  public static Command autoAim() {
    return new InstantCommand(AutoAimSubsystem::autoAim, RobotContainer.m_Arm);
  }
}
