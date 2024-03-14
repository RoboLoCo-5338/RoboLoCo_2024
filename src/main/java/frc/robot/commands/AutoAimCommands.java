package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

/** 
 * A class that probably needs to be removed and moved into another class unless we get methods
 * Anyways it does auto aim stuff
*/
public class AutoAimCommands {
  /**
   * Automatically moves arm to shoot
   * @return A command form of autoAim
   */
    public static Command autoAim() {
        return new InstantCommand(
          () -> RobotContainer.m_AutoAim.autoAim(),
          RobotContainer.m_Arm
        );
      }
}
