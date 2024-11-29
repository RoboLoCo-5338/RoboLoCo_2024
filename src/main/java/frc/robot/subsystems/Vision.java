package frc.robot.subsystems;

import frc.robot.Constants;

import java.util.HashSet;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.PhotonUtils;

public class Vision {
  private static PhotonCamera camera = new PhotonCamera("Rock");
  private static final PhotonPoseEstimator photonEstimator =
      new PhotonPoseEstimator(
          Constants.kTagLayout,
          PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          null); // TODO: Replace with the transform of the robot to the camera

  public static boolean hasResults() {
    return camera.getLatestResult().hasTargets();
  }

  public static double getTargetYaw() {
    if (hasResults()) return camera.getLatestResult().getBestTarget().getYaw();
    return 0;
  }

  public static double getTargetPitch() {
    if (hasResults()) return camera.getLatestResult().getBestTarget().getPitch();
    return 0;
  }

  public static double getTargetYaw(int aprilTag) {
    Optional<PhotonTrackedTarget> tag = getTagTarget(aprilTag);
    tag.ifPresent(target -> target.getYaw());
    return 0;
  }

  public static double getTargetPitch(int aprilTag) {
    Optional<PhotonTrackedTarget> tag = getTagTarget(aprilTag);
    tag.ifPresent(target -> target.getPitch());
    return 0;
  }
  
  public static double getDistanceToTarget(double targetHeight) {
    if (hasResults()) {
      return PhotonUtils.calculateDistanceToTargetMeters(
          Constants.CAMERA_HEIGHT_METERS,
          targetHeight,
          Constants.CAMERA_PITCH_RADIANS,
          getTargetPitch());
    }
    return -1;
  }

  public static int getBestTargetID() {
    if (hasResults()) return camera.getLatestResult().getBestTarget().getFiducialId();
    return -1;
  }

  public static Optional<PhotonTrackedTarget> getTagTarget(int tag){
    if(hasResults()) for(PhotonTrackedTarget target: camera.getLatestResult().getTargets()) if(target.getFiducialId()==tag) return Optional.of(target);
    return Optional.empty();
  }

  public static Optional<EstimatedRobotPose> getPose() {
    photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    return photonEstimator.update();
  }
}
