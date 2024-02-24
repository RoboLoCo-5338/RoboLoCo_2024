// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;


public class ArmCommands  {

  /** Creates a new ArmCommands. */
  public ArmCommands() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public static Command moveArm() {
    return new InstantCommand(
      () -> RobotContainer.m_Arm.moveArm(0.2),
      RobotContainer.m_Arm
    );
  }

  public static Command getArmPosition() {
    return new InstantCommand(
      () -> RobotContainer.m_Arm.getArmPosition(),
      RobotContainer.m_Arm
    );
  }
  
  public static Command setArm() {
    return new InstantCommand(
      () -> RobotContainer.m_Arm.setArm(0.2),
      RobotContainer.m_Arm
    );
  }

    public static Command stopArm() {
    return new InstantCommand(
      () -> RobotContainer.m_Arm.stopArm(),
      RobotContainer.m_Arm
    );
  }

}