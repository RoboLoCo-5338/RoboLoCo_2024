// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */



public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  public static final ArmSubsystem m_Arm = new ArmSubsystem();
  public static double percent = 0.3;
  public static int coneOffset = 0;

  public static int reverseModifier=1;

  // controllers
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);
  // public static Joystick controller1 = new Joystick(0); //driver
  // public static Joystick controller2 = new Joystick(1); //operator

  //   private static XboxController controller3 = new XboxController(0); potential driver controller stuff
  //   private static XboxController controller4 = new XboxController(1);

  public static JoystickButton limeLight;

  ChoreoTrajectory traj; //1/18/24
  Field2d m_field = new Field2d();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureButtonBindings();

    traj = Choreo.getTrajectory("Trajectory"); //1/18/24

    m_field.getObject("traj").setPoses(
    traj.getInitialPose(), traj.getFinalPose());
    m_field.getObject("trajPoses").setPoses(
      traj.getPoses());

        // SmartDashboard.putData(m_field);

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true, true),
            m_robotDrive));
  }

  private void configureButtonBindings() {
    // TODO fix: limeLight = new JoystickButton(controller1, Constants.ABUTTON);
  }

    /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // // Create config for trajectory
    // TrajectoryConfig config = new TrajectoryConfig(
    //     AutoConstants.kMaxSpeedMetersPerSecond,
    //     AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    //     // Add kinematics to ensure max speed is actually obeyed
    //     .setKinematics(DriveConstants.kDriveKinematics);

    // // An example trajectory to follow. All units in meters.
    // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
    //     // Start at the origin facing the +X direction
    //     new Pose2d(0, 0, new Rotation2d(0)),
    //     // Pass through these two interior waypoints, making an 's' curve path
    //     List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
    //     // End 3 meters straight ahead of where we started, facing forward
    //     new Pose2d(3, 0, new Rotation2d(0)),
    //     config);

    // ProfiledPIDController thetaController = new ProfiledPIDController(
    //     AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    // thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
    //     exampleTrajectory,
    //     m_robotDrive::getPose, // Functional interface to feed supplier
    //     DriveConstants.kDriveKinematics,

    //     // Position controllers
    //     new PIDController(AutoConstants.kPXController, 0, 0),
    //     new PIDController(AutoConstants.kPYController, 0, 0),
    //     thetaController,
    //     m_robotDrive::setModuleStates,
    //     m_robotDrive);

    // // Reset odometry to the starting pose of the trajectory.
    // m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    // return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, true, false));

    //1/18/24 ---- CHOREO TESTING

    PIDController thetaController = new PIDController(AutoConstants.kPThetaController, 0, 0);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    m_robotDrive.resetOdometry(traj.getInitialPose());

    Command swerveCommand = Choreo.choreoSwerveCommand(
        traj, // Choreo trajectory from above
        m_robotDrive::getPose, // A function that returns the current field-relative pose of the robot: your
                               // wheel or vision odometry
        new PIDController(Constants.AutoConstants.kPXController, 0.0, 0.0), // PIDController for field-relative X
                                                                                   // translation (input: X error in meters,
                                                                                   // output: m/s).
        new PIDController(Constants.AutoConstants.kPYController, 0.0, 0.0), // PIDController for field-relative Y
                                                                                   // translation (input: Y error in meters,
                                                                                   // output: m/s).
        thetaController, // PID constants to correct for rotation
                         // error
        (ChassisSpeeds speeds) -> m_robotDrive.drive( // needs to be robot-relative
            speeds.vxMetersPerSecond,
            speeds.vyMetersPerSecond,
            speeds.omegaRadiansPerSecond,
            false, true),
        () -> {
            return false;
          }, // Whether or not to mirror the path based on alliance (CAN ADD LOGIC TO DO THIS AUTOMATICALLY)
        m_robotDrive // The subsystem(s) to require, typically your drive subsystem only
    );

    return Commands.sequence(
      Commands.runOnce(() -> m_robotDrive.resetOdometry(traj.getInitialPose())),
      swerveCommand,
      m_robotDrive.run(() -> m_robotDrive.drive(0, 0, 0, false, true))
  );
  }
  // DriverStation.Alliance ally = DriverStation.getAlliance();
  //   if (ally == DriverStation.Alliance.Red) {
  //     switch (AutoConstants.autoNum){
  //       case 1:
  //         return AutoCommands.leftRed();
  //       case 2:
  //         return AutoCommands.midRed();
  //       case 3:
  //         return AutoCommands.rightRed();
  //     }
  //   }
  //   else if (ally == DriverStation.Alliance.Blue) {
  //     switch (AutoConstants.autoNum){
  //       case 1:
  //         return AutoCommands.leftBlue();
  //       case 2:
  //         return AutoCommands.midBlue();
  //       case 3:
  //         return AutoCommands.rightBlue();
  //     }
  //   }
  //   else {
  //       return null;
  //   }
  //   return null;

   public static ParallelCommandGroup moveMechanismPID(int preset){
    return new ParallelCommandGroup(
      // ElevatorCommands.setElevatorHeight(preset), 
      // new SetArmAbsolute(preset)
     );
   }
}
