// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

   // CameraServer.startAutomaticCapture();
    //Run a warmup to to warm up the relevant libraries
    FollowPathCommand.warmupCommand().schedule();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
     SmartDashboard.putBoolean("LaserCan is Note", RobotContainer.m_Intake.isNote());
    //SmartDashboard.putNumber("LaserCAN Measurment(Meters)", RobotContainer.m_Intake.getLaserCanMeasurement().distance_mm/1000.0);

    SmartDashboard.putNumber("Heading",RobotContainer.m_robotDrive.getHeading());
    SmartDashboard.putNumber("X Pose", RobotContainer.m_robotDrive.getPose().getX());
    SmartDashboard.putNumber("Y Pose", RobotContainer.m_robotDrive.getPose().getY());
    SmartDashboard.putNumber("Angle", RobotContainer.m_robotDrive.m_gyro.getAngle());
    SmartDashboard.putNumber("Front left speed", RobotContainer.m_robotDrive.m_frontLeft.getState().speedMetersPerSecond);
    SmartDashboard.putNumber("Front right speed", RobotContainer.m_robotDrive.m_frontRight.getState().speedMetersPerSecond);
    SmartDashboard.putNumber("Rear left speed", RobotContainer.m_robotDrive.m_rearLeft.getState().speedMetersPerSecond);
    SmartDashboard.putNumber("Rear right speed", RobotContainer.m_robotDrive.m_rearRight.getState().speedMetersPerSecond);

    SmartDashboard.putNumber("Front left distance (meters)", RobotContainer.m_robotDrive.m_frontLeft.getPosition().distanceMeters);
    SmartDashboard.putNumber("Front right distance (meters)", RobotContainer.m_robotDrive.m_frontRight.getPosition().distanceMeters);
    SmartDashboard.putNumber("Rear left distance (meters)", RobotContainer.m_robotDrive.m_rearLeft.getPosition().distanceMeters);
    SmartDashboard.putNumber("Rear right distance (meters)", RobotContainer.m_robotDrive.m_rearRight.getPosition().distanceMeters);

    SmartDashboard.putNumber("Arm Encoder Value", RobotContainer.m_Arm.getArmPosition());



    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    //m_autonomousCommand = IntakeCommands.runIntakeUntilNoteSequentialCommand();

    //m_autonomousCommand = RobotContainer.m_robotDrive.steer_sysIdQuasistatic(SysIdRoutine.Direction.kReverse);
   m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    //m_autonomousCommand = AutoCommands.shootAuto();
    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    RobotContainer.m_Arm.resetArm();
    RobotContainer.m_Arm.stopArm();
    RobotContainer.m_robotDrive.drive(0, 0, 0, false, false, false); //added 6/1/24 to straighten wheels upon teleop init for testing

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

   // SmartDashboard.putBoolean("Isnote", IntakeSubsystem.noteInside);
    //  if(RobotContainer.m_Intake.isNote()){

    //   if(!IntakeSubsystem.noteInside){
    //      CommandScheduler.getInstance().schedule(m_robotContainer.rumbleGamePad(500));
      
    //   }
    //   IntakeSubsystem.noteInside=true;
    // }else{
    //   IntakeSubsystem.noteInside=false;
    // }
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
