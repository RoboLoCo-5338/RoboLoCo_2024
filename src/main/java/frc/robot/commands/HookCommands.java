package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Hook;

public class HookCommands {
    public static Command moveUp() {
        return new InstantCommand(
        () -> RobotContainer.m_hook.moveHook(1),
        RobotContainer.m_hook
        );
    }
    public static Command moveDown() {

        return new InstantCommand(
            () -> RobotContainer.m_hook.moveHook(-1),
            RobotContainer.m_hook
            );
    }

    public static Command moveHook(double speed) {
        return new InstantCommand(
            () -> RobotContainer.m_hook.moveHook(speed),
            RobotContainer.m_hook
            );

    }

    public static Command stopHook() {
         return new InstantCommand(
            () -> RobotContainer.m_hook.stopHook(),
            RobotContainer.m_hook
            );
    }
}
