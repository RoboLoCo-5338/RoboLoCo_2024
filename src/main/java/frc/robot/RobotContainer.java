// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.DriverStation;

import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.*;
import frc.robot.commands.*;

// Starts recording to data log

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static final ArmSubsystem m_Arm = new ArmSubsystem();
  public static final ShooterSubsystem m_shooter = new ShooterSubsystem();
  public static final IntakeSubsystem m_Intake = new IntakeSubsystem();
  public static final AutoAimSubsystem m_AutoAim = new AutoAimSubsystem();
  public static final DriveSubsystem m_robotDrive = new DriveSubsystem();

  // controllers
  public static final CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  public static final CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);

  public static boolean slowMode = false;

  //ChoreoTrajectory traj; // 1/18/24
  Field2d m_field = new Field2d();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    NamedCommands.registerCommand("intake", IntakeCommands.moveIntakeIn());
   
    configureButtonBindings();
    DataLogManager.start();

    // Record both DS control and joystick data
    DriverStation.startDataLog(DataLogManager.getLog());

    // Configure default commands
    m_robotDrive.setDefaultCommand(
      // The left stick controls translation of the robot.
      // Turning is controlled by the X axis of the right stick.
      new RunCommand(() -> m_robotDrive.drive(-MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                                              -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                                              -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                                              true, true),
                     m_robotDrive));
  }

  public Command makeRobotSlow() {
    return new InstantCommand(() -> slowMode = !slowMode );
  }

  public Command resetGyroTeleop() {
    return new InstantCommand(RobotContainer.m_robotDrive.m_gyro::reset);
  }
  
  private void configureButtonBindings() {
    Trigger ampPreset = new Trigger(m_operatorController.y());
    ampPreset.onTrue(ArmCommands.setArm(51));

    Trigger climbPreset = new Trigger(m_operatorController.b());
    climbPreset.onTrue(ArmCommands.setArm(16));

    Trigger restPreset = new Trigger(m_operatorController.a());
    restPreset.onTrue(ArmCommands.setArm(0));

    // Trigger makeRobotSlow = new Trigger(m_driverController.rightTrigger());
    // makeRobotSlow.onTrue(makeRobotSlow());

    Trigger moveArmUp = new Trigger(() -> m_operatorController.getLeftY()> OIConstants.kArmDeadband);
    moveArmUp.whileTrue(ArmCommands.moveArmUp());

    Trigger moveArmDown = new Trigger(() -> m_operatorController.getLeftY()< -OIConstants.kArmDeadband);
    moveArmDown.whileTrue(ArmCommands.moveArmDown());

    Trigger stopArm = new Trigger(() -> Math.abs(m_operatorController.getLeftY())<OIConstants.kArmDeadband);
    stopArm.whileTrue(ArmCommands.stopArm());

    Trigger indexFast = new Trigger(()-> m_operatorController.getRightY()>OIConstants.kArmDeadband);
    Trigger smallerDistance = new Trigger(() -> m_Intake.getLaserCanMeasurement().distance_mm/1000<5);
    indexFast.and(smallerDistance.negate()).whileTrue(IntakeCommands.moveIndexerInFast());

    Trigger ampArm = new Trigger(m_operatorController.y());

    Trigger climbArm = new Trigger(m_operatorController.b());

    Trigger intakeIn = new Trigger(m_operatorController.rightTrigger());
    intakeIn.whileTrue(IntakeCommands.moveIntakeIn());
    intakeIn.onFalse(IntakeCommands.stopIntake());

    Trigger intakeOut = new Trigger(m_operatorController.leftTrigger());
    intakeOut.whileTrue(IntakeCommands.moveIntakeOut());
    intakeOut.onFalse(IntakeCommands.stopIntake());

    // Trigger autoAim = new Trigger(m_operatorController.b());
    // autoAim.whileTrue(AutoAimCommands.autoAim());

    Trigger shootOut = new Trigger(m_operatorController.rightBumper());
    shootOut.whileTrue(ShooterCommands.shooterForward());
    shootOut.onFalse(ShooterCommands.shooterStop());

    Trigger shootIn = new Trigger(m_operatorController.leftBumper());
    shootIn.whileTrue(ShooterCommands.shooterReverse());
    shootIn.onFalse(ShooterCommands.shooterStop());

    // reset gyro button binding 
    Trigger resetGyro = new Trigger(m_driverController.x());
    resetGyro.onTrue(resetGyroTeleop());

    // turnToTagCommand().execute();
    // m_Arm.doAutoAim(Constants.RobotTarget.SPEAKER).execute();

    // check if joystick and buttons are NOT being pressed
    // if they aren't being pressed, set speed to zero
    // elsewise, take input from joystick and buttons
    // joystick > buttons
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // everything below this point was in getautocommands
  // // Create config for trajectory
  // TrajectoryConfig config = new TrajectoryConfig(
  // AutoConstants.kMaxSpeedMetersPerSecond,
  // AutoConstants.kMaxAccelerationMetersPerSecondSquared)
  // // Add kinematics to ensure max speed is actually obeyed
  // .setKinematics(DriveConstants.kDriveKinematics);

  // // An example trajectory to follow. All units in meters.
  // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
  // // Start at the origin facing the +X direction
  // new Pose2d(0, 0, new Rotation2d(0)),
  // // Pass through these two interior waypoints, making an 's' curve path
  // List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
  // // End 3 meters straight ahead of where we started, facing forward
  // new Pose2d(3, 0, new Rotation2d(0)),
  // config);

  // ProfiledPIDController thetaController = new ProfiledPIDController(
  // AutoConstants.kPThetaController, 0, 0,
  // AutoConstants.kThetaControllerConstraints);
  // thetaController.enableContinuousInput(-Math.PI, Math.PI);

  // SwerveControllerCommand swerveControllerCommand = new
  // SwerveControllerCommand(
  // exampleTrajectory,
  // m_robotDrive::getPose, // Functional interface to feed supplier
  // DriveConstants.kDriveKinematics,

  // // Position controllers
  // new PIDController(AutoConstants.kPXController, 0, 0),
  // new PIDController(AutoConstants.kPYController, 0, 0),
  // thetaController,
  // m_robotDrive::setModuleStates,
  // m_robotDrive);

  // // Reset odometry to the starting pose of the trajectory.
  // m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

  // Run path following command, then stop at the end.
  // return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0,
  // true, false));
}
