package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;

public class IntakeCommands {
    public static Command intake(double speed){
        return new InstantCommand(
        () -> RobotContainer.intake.moveIntake(speed),
        RobotContainer.intake
        );

    }

    public static Command stopIntake() {
        return new InstantCommand(
            () -> RobotContainer.intake.moveIntake(0),
            RobotContainer.intake
            );

    }


}



