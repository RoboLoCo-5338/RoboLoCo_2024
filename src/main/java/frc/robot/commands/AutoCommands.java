package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.RobotContainer;
import com.choreo.lib.Choreo; //1/18/24
import com.choreo.lib.ChoreoTrajectory;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;


public class AutoCommands {
  static ChoreoTrajectory traj; //1/18/24
  static Field2d m_field = new Field2d();
  static DriveSubsystem m_robotDrive = RobotContainer.getDriveSystem();
    public static Command pathPlannerTest(){
      return new PathPlannerAuto("AUTO Name");
    }
    public static Command leftRed(){
      return null;
      //Add code
    }
    public static Command dumbAuto() {
      return runTrajectory("DumbTraj");
    }

    public static Command midRed(){
      return runTrajectory("NewPath");
    }
    public static Command rightRed(){
      return null;
      //Add code
    }
    public static Command leftBlue(){
      return null;
      //Add code
    }
    public static Command midBlue(){
      return null;
      //Add code
    }
    public static Command rightBlue(){
      return null;
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
}