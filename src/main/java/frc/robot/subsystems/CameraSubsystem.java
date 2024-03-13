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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.OIConstants;
import frc.robot.RobotContainer;

import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.hardware.Pigeon2;

public class CameraSubsystem {
    final static double ANGULAR_P = 0.1;
    final static double ANGULAR_D = 0.0;
    static PhotonCamera logcamera = new PhotonCamera("LogitechCam"); //Change the name when needed or things will break
    

    public static Command turnToNote(){ //untested
        if(!logcamera.getLatestResult().hasTargets()){
        return null;
        }
        else{
        return Commands.sequence(
            RobotContainer.m_robotDrive.runOnce(() -> RobotContainer.m_robotDrive.drive(
            -MathUtil.applyDeadband(RobotContainer.m_driverController.getLeftY(), OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(RobotContainer.m_driverController.getLeftX(), OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(new PIDController(ANGULAR_P,0,ANGULAR_D).calculate(getTargetYaw(false)), OIConstants.kDriveDeadband),
            true, true)
            )
        );
        }
  }
      public static double distanceFromNote(double targetHeight){
        if(logcamera.getLatestResult().hasTargets()){
            return PhotonUtils.calculateDistanceToTargetMeters(Constants.LOGITECH_CAMERA_HEIGHT_METERS, targetHeight, Constants.LOGTECH_CAMERA_PITCH_RADIANS, getTargetPitch(true));
        }
        else{
            return Double.POSITIVE_INFINITY;
        }
    }
    public static double getTargetYaw(boolean radians){ //The boolean is if you want or don't want radians
        if(logcamera.getLatestResult().hasTargets()){
            double output=logcamera.getLatestResult().getBestTarget().getYaw();
            if(radians){
                output=Units.degreesToRadians(output);
            }
            return output;
        }
        return Double.POSITIVE_INFINITY;
    }
    public static double getTargetPitch(boolean radians){ //The boolean is if you want or don't want radians
        if(logcamera.getLatestResult().hasTargets()){
            double output=logcamera.getLatestResult().getBestTarget().getPitch();
            if(radians){
                output=Units.degreesToRadians(output);
            }
            return output;
        }
        return Double.POSITIVE_INFINITY;
    }
    // public static Command driveToNote(){
    //     if(logcamera.getLatestResult().hasTargets()){
    //         Transform3d bestCameraToTarget = logcamera.getLatestResult().getBestTarget().getBestCameraToTarget();
    //         RobotContainer.m_robotDrive.runOnce(() -> RobotContainer.m_robotDrive.drive(
    //         distanceFromNote()*Math.cos(MathUtil.degreesToRadians(m_gyro.getDegrees)),
    //         distanceFromNote()*Math.sin(MathUtil.degreesToRadians(m_gyro.getDegrees))
    //     )
    //     );

    //     }
    //     return null;
    // }
}