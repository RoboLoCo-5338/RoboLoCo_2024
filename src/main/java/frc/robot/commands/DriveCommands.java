// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Vision;

/** Add your docs here. */
public class DriveCommands {

    public static long startTime;

    public static Command driveForwardTimed(double time,double speed){
        return new FunctionalCommand(() -> {
            RobotContainer.m_robotDrive.driveSpeed(0.0);
            startTime= System.currentTimeMillis();
        }, 
        ()-> RobotContainer.m_robotDrive.driveSpeed(speed),
     interrupted -> RobotContainer.m_robotDrive.driveSpeed(0),
        ()-> System.currentTimeMillis()-time>startTime,
         RobotContainer.m_robotDrive);
    }

    public static Command driveForewardUntilIntake(double speed){
        return new FunctionalCommand(() -> {
            RobotContainer.m_robotDrive.driveSpeed(0);
            RobotContainer.m_Intake.stopIntakeIndexer();
          }, () -> {RobotContainer.m_robotDrive.driveRobotRelative(new ChassisSpeeds(speed, 0, 0)); RobotContainer.m_Intake.inIntakeIndexer();}, 
          interrupted -> {RobotContainer.m_robotDrive.driveSpeed(0); RobotContainer.m_Intake.stopIntakeIndexer();}, 
            () -> RobotContainer.m_Intake.isNote(), RobotContainer.m_robotDrive, RobotContainer.m_Intake);
    }

    public static Command driveToTag(double speed){
        return new FunctionalCommand(() -> {RobotContainer.m_robotDrive.driveSpeed(0);}, 
       () -> {RobotContainer.m_robotDrive.driveRobotRelative(new ChassisSpeeds(speed, 0, 0));}, interrupted -> RobotContainer.m_robotDrive.driveSpeed(0), () ->Vision.distanceFromTarget(Constants.aprilTagHeights[1])<5, RobotContainer.m_robotDrive);
    }


}
