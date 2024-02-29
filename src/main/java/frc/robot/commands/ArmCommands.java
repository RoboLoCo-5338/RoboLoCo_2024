// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;


public class ArmCommands  {

  /** Creates a new ArmCommands. */
  public ArmCommands() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public static Command moveArmUp() {
    return new InstantCommand(
      () -> RobotContainer.m_Arm.moveArmUp(),
      RobotContainer.m_Arm
    );
  }

    public static Command moveArmDown() {
    return new InstantCommand(
      () -> RobotContainer.m_Arm.moveArmDown(),
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