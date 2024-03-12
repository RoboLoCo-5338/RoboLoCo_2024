package frc.robot.subsystems;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.OIConstants;
import frc.robot.RobotContainer;

import org.photonvision.targeting.PhotonTrackedTarget;

public class CameraSubsystem {
    static PhotonCamera camera = new PhotonCamera("LogitechCam"); //Change the name when needed or things will break

    public static Command turnToNote(){ //untested
        final double ANGULAR_P = 0.1;
        final double ANGULAR_D = 0.0;
        if(!Vision.hasResults()){
        return null;
        }
        else{
        return Commands.sequence(
            RobotContainer.m_robotDrive.runOnce(() -> RobotContainer.m_robotDrive.drive(
            -MathUtil.applyDeadband(RobotContainer.m_driverController.getLeftY(), OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(RobotContainer.m_driverController.getLeftX(), OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(new PIDController(ANGULAR_P,0,ANGULAR_D).calculate(Vision.getTargetYaw(false)), OIConstants.kDriveDeadband),
            true, true)
            )
        );
        }
  }
}
