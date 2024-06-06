package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.RobotContainer;

import java.util.List;
import java.util.Optional;

import com.choreo.lib.Choreo; //1/18/24
import com.choreo.lib.ChoreoTrajectory;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public final class AutoCommands {
  private static ChoreoTrajectory trajectory; //1/18/24
  private static final Field2d m_field = new Field2d();
  private static final DriveSubsystem m_robotDrive = RobotContainer.m_robotDrive;

  public static Command pathPlannerTest() {
    PathPlannerPath path = PathPlannerPath.fromPathFile("straight_line_3m_middle");

    return new SequentialCommandGroup(
      new InstantCommand(() -> m_robotDrive.resetOdometry(getPathPose(path))),
      new WaitCommand(0.1),
      AutoBuilder.followPath(path));
  }

  public static Pose2d getPathPose(PathPlannerPath pPath) {
    Optional<Alliance> alliance = DriverStation.getAlliance();

    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
      return pPath.flipPath().getPreviewStartingHolonomicPose();
    }

    return pPath.getPreviewStartingHolonomicPose();
  }
    
  public static Command leftRed() {
    return null;
    //Add code
  }

  public static Command dumbAuto() {
    return runTrajectory("DumbTraj");
  }

  public static Command midRed() {
    return runTrajectory("NewPath");
  }

  public static Command rightRed() {
    return null;
    //Add code
  }

  public static Command leftBlue() {
    return null;
    //Add code
  }

  public static Command midBlue() {
    return null;
    //Add code
  }

  public static Command rightBlue() {
    return null;
    //Add code
  }

  public static Command runTrajectory(String name) {
    trajectory = Choreo.getTrajectory(name); //1/18/24

    m_field.getObject("traj").setPoses(trajectory.getInitialPose(), trajectory.getFinalPose());
    m_field.getObject("trajPoses").setPoses(trajectory.getPoses());

    PIDController thetaController = new PIDController(AutoConstants.kPThetaController, 0, 0);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    m_robotDrive.resetOdometry(trajectory.getInitialPose());

    Command swerveCommand = Choreo.choreoSwerveCommand(
            trajectory, // Choreo trajectory from above
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
      (ChassisSpeeds speeds) -> m_robotDrive.drive(speeds.vxMetersPerSecond,// needs to be robot-relative
                                                   speeds.vyMetersPerSecond,
                                                   speeds.omegaRadiansPerSecond,
                                                   false, true),
      () -> false, // Whether or not to mirror the path based on alliance (CAN ADD LOGIC TO DO THIS AUTOMATICALLY)
      m_robotDrive); // The subsystem(s) to require, typically your drive subsystem only

   return Commands.sequence(Commands.runOnce(() -> m_robotDrive.resetOdometry(trajectory.getInitialPose())),
                            swerveCommand,
                            m_robotDrive.run(() -> m_robotDrive.drive(0, 0, 0, false, true)));
  }

  public static Command shootAuto() {
      return new ParallelCommandGroup(ShooterCommands.runShooterForwardTimed(1500),
                                      new SequentialCommandGroup(new WaitCommand(0.75),
                                                                 IntakeCommands.runIntakeForwardTimed(750)));
  }

  public static Command intakeInAuto() {
    return new ParallelCommandGroup(IntakeCommands.runIntakeForwardTimed(1000));
  }

  // YALL PLEASE USE THIS AUTO FOR COMP
  public static Command realauto() {
    return new SequentialCommandGroup(shootAuto(),
                                      new WaitCommand(6),
                                      new ParallelCommandGroup(pathPlannerTest(), IntakeCommands.runIntakeForwardTimed(6000)));
      
  }

  public static Command driveForwardAutoComp() {
    return DriveCommands.driveForwardTimed(2000, 0.4);
  }

  public static Command driveForward() {
    // shootAuto();
    TrajectoryConfig config = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
                                                   AutoConstants.kMaxAccelerationMetersPerSecondSquared)
      .setKinematics(DriveConstants.kDriveKinematics); // Add kinematics to ensure max speed is actually obeyed

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
      // Start at the origin facing the +X direction
      new Pose2d(0, 0, new Rotation2d(0)),

      // Pass through these two interior waypoints, making an 's' curve path
      List.of(new Translation2d(1.5, 0)),

      // End 3 meters straight ahead of where we started, facing forward
      new Pose2d(3, 0, new Rotation2d(0)),

      config);

    ProfiledPIDController thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0,
                                                                      AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
      exampleTrajectory,
      m_robotDrive::getPose, // Functional interface to feed supplier
      DriveConstants.kDriveKinematics,

      // Position controllers
      new PIDController(AutoConstants.kPXController, 0, 0),
      new PIDController(AutoConstants.kPYController, 0, 0),

      thetaController,
      m_robotDrive::setModuleStates,
      m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    //Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, true, false));
  }

  public static Command straightlinetest() {
    return new SequentialCommandGroup(runTrajectory("straightline1m"));
  }

  public static Command getPath(String name) {
    PathPlannerPath path = PathPlannerPath.fromPathFile(name);
    return AutoBuilder.followPath(path);
  }

  public static SequentialCommandGroup center3NoteAuto() {
    PathPlannerPath path = PathPlannerPath.fromPathFile("start_center_straight");

    return new SequentialCommandGroup(
      new InstantCommand(() -> m_robotDrive.resetOdometry(getPathPose(path))),
      new WaitCommand(0.1),
      new InstantCommand(AutoCommands::shootAuto),
      new ParallelDeadlineGroup(AutoBuilder.followPath(path), IntakeCommands.runIntakeOnlyTimed(6000)),
      new ParallelCommandGroup(new SequentialCommandGroup(getPath("2_center_straight"),
                                                          IntakeCommands.runIntakeForwardTimed(1500),
                                                          new WaitCommand(0.5)),
                               ShooterCommands.runShooterForwardTimed(9000)));
  }

  public static SequentialCommandGroup center4NoteAuto() {
    PathPlannerPath path = PathPlannerPath.fromPathFile("start_center_straight");
    return new SequentialCommandGroup(
      new InstantCommand(() -> m_robotDrive.resetOdometry(getPathPose(path))),
      new InstantCommand(AutoCommands::shootAuto),
      new ParallelRaceGroup(AutoBuilder.followPath(path), IntakeCommands.runIntakeUntilNote()),
      new ParallelRaceGroup(new SequentialCommandGroup(getPath("2_center_straight"),
                                                       IntakeCommands.runIntakeForwardTimed(1000)),
                            ShooterCommands.runShooterForwardTimed(9000)),
      new ParallelRaceGroup(getPath("3_center_straight"), IntakeCommands.runIntakeUntilNote()),
      new ParallelRaceGroup(new SequentialCommandGroup(getPath("4_center_straight"), IntakeCommands.runIntakeForwardTimed(1000)),
                            ShooterCommands.runShooterForwardTimed(9000)),
      new ParallelRaceGroup(getPath("5_center_straight"), IntakeCommands.runIntakeUntilNote()),
      new ParallelRaceGroup(new SequentialCommandGroup(getPath("6_center_straight"),
                                                       IntakeCommands.runIntakeForwardTimed(1000))),
      new InstantCommand(() -> ShooterCommands.runShooterForwardTimed(9000)));
  }

  public static Command getAutonomousCommand() {
    return center3NoteAuto();
  }
}
