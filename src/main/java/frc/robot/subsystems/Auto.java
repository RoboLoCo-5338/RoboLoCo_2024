package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.AutoCommands;

import java.util.Optional;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
public class Auto {
    private static int autoNum;
    private static SendableChooser<Integer> m_chooser = new SendableChooser<>();
    public static void autoChooser(){
        m_chooser.setDefaultOption("Left", 1);
        m_chooser.addOption("Middle", 2);
        m_chooser.addOption("Right", 3);
        SmartDashboard.putData("Auto Choices", m_chooser);
    }
    public static void autoSelect(){
    //   autoNum=m_chooser.getSelected();
    }


    public static Command returnChoreoCommand(String name){

    PIDController thetaController = new PIDController(AutoConstants.kPThetaController, 0, 0);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    ChoreoTrajectory traj = Choreo.getTrajectory(name); // 1/18/24
    Field2d m_field = new Field2d();
    m_field.getObject("traj").setPoses(
        traj.getInitialPose(), traj.getFinalPose());
    m_field.getObject("trajPoses").setPoses(
        traj.getPoses());
      Command swerveCommand = Choreo.choreoSwerveCommand(
        traj, // Choreo trajectory from above
       RobotContainer.m_robotDrive::getPose, // A function that returns the current field-relative pose of the robot: your
                               // wheel or vision odometry
        new PIDController(Constants.AutoConstants.kPXController, 0.0, 0.0), // PIDController for field-relative X
                                                                            // translation (input: X error in meters,
                                                                            // output: m/s).
        new PIDController(Constants.AutoConstants.kPYController, 0.0, 0.0), // PIDController for field-relative Y
                                                                            // translation (input: Y error in meters,
                                                                            // output: m/s).
        thetaController, // PID constants to correct for rotation
                         // error
        (ChassisSpeeds speeds) -> RobotContainer.m_robotDrive.drive( // needs to be robot-relative
            speeds.vxMetersPerSecond,
            speeds.vyMetersPerSecond,
            speeds.omegaRadiansPerSecond,
            false, true),
        () -> {
          return false;
        }, 
          RobotContainer.m_robotDrive
      );
     return Commands.sequence(
        Commands.runOnce(() -> RobotContainer.m_robotDrive.resetOdometry(traj.getInitialPose())),
        swerveCommand,
        RobotContainer.m_robotDrive.run(() -> RobotContainer.m_robotDrive.drive(0, 0, 0, true, true)));
    }
    public static Command getAutonomousCommand() {
       
      return new PathPlannerAuto("Straight_Line");
    
    }
}