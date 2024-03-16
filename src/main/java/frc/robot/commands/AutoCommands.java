package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.*;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
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

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;


public class AutoCommands {
  static ChoreoTrajectory traj; //1/18/24
  static Field2d m_field = new Field2d();
  
  static boolean red=(DriverStation.getAlliance().get() == Alliance.Red);
  static DriveSubsystem m_robotDrive = RobotContainer.m_robotDrive;
    // public static Command leftRed(){
    //    return runTrajectory("bottom_red_leave");
    //   //Add code
    // }
    // public static Command midRed(){
    //   return runTrajectory("middle_red_leave");
    // }
    // public static Command rightRed(){
    //   return runTrajectory("top_red_leave");
    //   //Add code
    // }
    public static Command left(){
      return runTrajectory("top_blue_leave");
      //Add code
    }
    public static Command mid(){
       return runTrajectory("middle_blue_leave");
      //Add code
    }
    public static Command right(){
      return runTrajectory("bottom_blue_leave");
      //Add code
    }
    public static Command runTrajectory(String name){
      traj = Choreo.getTrajectory(name); //1/18/24

      m_field.getObject("traj").setPoses(
      traj.getInitialPose(), traj.getFinalPose()
      );
      m_field.getObject("trajPoses").setPoses(
        traj.getPoses()
      );
      var thetaController = new PIDController(AutoConstants.kPThetaController, 0, 0);
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
        (ChassisSpeeds speeds) -> m_robotDrive.autoDrive( // needs to be robot-relative
            speeds.vxMetersPerSecond,
            speeds.vyMetersPerSecond,
            speeds.omegaRadiansPerSecond
          ),
        () -> {
            return red;
          }, // Whether or not to mirror the path based on alliance (CAN ADD LOGIC TO DO THIS AUTOMATICALLY)
        m_robotDrive // The subsystem(s) to require, typically your drive subsystem only
    );

    return Commands.sequence(
      Commands.runOnce(() -> m_robotDrive.resetOdometry(traj.getInitialPose())),
      swerveCommand,
      m_robotDrive.run(() -> m_robotDrive.drive(0, 0, 0,false,false))
      
     );
    // return null;
    }

    public static Command shootAuto(){
 

      return new ParallelCommandGroup(
        ShooterCommands.runShooterForwardTimed(1500),
        new SequentialCommandGroup(new WaitCommand(0.75), 
        IntakeCommands.runIntakeForwardTimed(750))
    
      );
     
      
    }

    // YALL PLEASE USE THIS AUTO FOR COMP 
    public static Command realauto(){
      return new SequentialCommandGroup(
        shootAuto(),
        new WaitCommand(5)
        
      );
      
    }

    public static Command driveForwardAutoComp(){
      return DriveCommands.driveForwardTimed(2000, 0.4);
    }


    public static Command driveForward(){

      // shootAuto();

    TrajectoryConfig config = new TrajectoryConfig(
    AutoConstants.kMaxSpeedMetersPerSecond,
    AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    // Add kinematics to ensure max speed is actually obeyed
    .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
    // Start at the origin facing the +X direction
    new Pose2d(0, 0, new Rotation2d(0)),
    // Pass through these two interior waypoints, making an 's' curve path
    List.of(new Translation2d(1.5, 0)),
    // End 3 meters straight ahead of where we started, facing forward
    new Pose2d(3, 0, new Rotation2d(0)),
    config);

    ProfiledPIDController thetaController = new ProfiledPIDController(
    AutoConstants.kPThetaController, 0, 0,
    AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new
    SwerveControllerCommand(
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
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0,
    true, false));
    }
    
    public static Command straightlinetest(){
      return new SequentialCommandGroup(
        runTrajectory("straightlinetestreal")
      );
    }

}
