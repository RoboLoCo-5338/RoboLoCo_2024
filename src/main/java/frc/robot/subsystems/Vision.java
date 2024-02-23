package frc.robot.subsystems;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import org.photonvision.targeting.PhotonTrackedTarget;

import frc.robot.Constants;
// Speaker id's are 4 (red) 7 (blue)

public class Vision {
    static AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(); // this is the coolest method ever
    static PhotonCamera camera = new PhotonCamera("Captain Rivets"); //Change the name when needed or things will break
    // notes about robotToCam
    // the Translation3d is from the CENTER of the robot
    // https://drive.google.com/file/d/14ehBwUCPgZg1t8bk1RZi0HZHaisDKb0k/view?usp=sharing very professional diagram
    static Transform3d robotToCam = new Transform3d(new Translation3d(Constants.X_OFFSET_CAMERA_TO_PIVOT, Constants.Y_OFFSET_CAMERA_TO_PIVOT, Constants.Z_OFFSET_CAMERA_TO_PIVOT), new Rotation3d(0,0,0));
    static PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.LOWEST_AMBIGUITY , camera, robotToCam);
    // Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), aprilTagFieldLayout.getTagPose(target.getFiducialId()), cameraToRobot);
    //TODO change PoseStrategy from LOWEST_AMBIGUITY to MULTI_TAG_PNP_ON_COPROCESSOR
    //https://docs.photonvision.org/en/latest/docs/programming/photonlib/robot-pose-estimator.html
    // apparently it improves performance but needs some setup and im not about doing setup rn


    public Vision(){}
    public static boolean hasResults(){
        return camera.getLatestResult().hasTargets();
    }
    public static PhotonTrackedTarget getBestTarget(){
        return camera.getLatestResult().getBestTarget();
    }
    public static Optional<Pose3d> getPoseRelativeToAprilTag() {
        if (hasResults()) { 
            PhotonTrackedTarget target = camera.getLatestResult().getBestTarget();
            if (target.getFiducialId() == 7 || target.getFiducialId() == 4) {
                Pose3d tagPose = aprilTagFieldLayout.getTagPose(target.getFiducialId()).get();
                Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), tagPose, robotToCam);
                Pose3d difference = tagPose.relativeTo(robotPose);
                return Optional.of(difference);
            }
        }
        return Optional.empty();
       
    }
    public static double getTargetPitch(boolean radians){ //The boolean is if you want or don't want radians
        if(hasResults()){
            double output=getBestTarget().getPitch();
            if(radians){
                output=Units.degreesToRadians(output);
            }
            return output;
        }
        return Double.POSITIVE_INFINITY;
    }public static double getTargetYaw(boolean radians){ //The boolean is if you want or don't want radians
        if(hasResults()){
            double output=getBestTarget().getYaw();
            if(radians){
                output=Units.degreesToRadians(output);
            }
            return output;
        }
        return Double.POSITIVE_INFINITY;
    }
    public static double distanceFromTarget(double targetHeight){
        if(hasResults()){
            return PhotonUtils.calculateDistanceToTargetMeters(Constants.CAMERA_HEIGHT_METERS, targetHeight, Constants.CAMERA_PITCH_RADIANS, getTargetPitch(true));
        }
        else{
            return Double.POSITIVE_INFINITY;
        }
    }
}
