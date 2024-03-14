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
  /**
   * Moves the arm up
   * @return A command to move the arm up
   */
  public static Command moveArmUp() {
    return new InstantCommand(
      () -> RobotContainer.m_Arm.moveArmUp(),
      RobotContainer.m_Arm
    );
  }
  /**
   * Moves the arm down
   * @return A command to move the arm down
   */
    public static Command moveArmDown() {
    return new InstantCommand(
      () -> RobotContainer.m_Arm.moveArmDown(),
      RobotContainer.m_Arm
    );
  }

  /**
   * Gets the arm position in rotations
   * @return Position of arm in rotations
   */
  public static Command getArmPosition() {
    return new InstantCommand(
      () -> RobotContainer.m_Arm.getArmPosition(),
      RobotContainer.m_Arm
    );
  }
  /**
   * Moves the position of the arm in rotations
   * @param position Position of the arm in rotations
   * @return Command to move the position of the arm
   */  
  public static Command setArm(double position) {
    return new InstantCommand(
      () -> RobotContainer.m_Arm.setArm(position),
      RobotContainer.m_Arm
    );
  }
  /**
   * Stops the arm
   * @return A command to stop the arm
   */
    public static Command stopArm() {
    return new InstantCommand(
      () -> RobotContainer.m_Arm.stopArm(),
      RobotContainer.m_Arm
    );
  }

}