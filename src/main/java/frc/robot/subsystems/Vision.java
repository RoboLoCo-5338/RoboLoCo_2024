package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants.VisionConstants;

import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision {
	private static PhotonCamera camera = new PhotonCamera("Rock");
	private static final PhotonPoseEstimator photonEstimator = new PhotonPoseEstimator(VisionConstants.kTagLayout,
			PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, null); // TODO: Replace with the transform of the robot to the
																// camera

	public static boolean hasResults() {
		return camera.getLatestResult().hasTargets();
	}

	public static double getTargetYaw() {
		if (hasResults())
			return camera.getLatestResult().getBestTarget().getYaw();
		return 0;
	}

	public static double getTargetPitch() {
		if (hasResults())
			return camera.getLatestResult().getBestTarget().getPitch();
		return 0;
	}

	public static double getTargetYaw(int aprilTag) {
		Optional<PhotonTrackedTarget> tag = getTagTarget(aprilTag);
		if (tag.isPresent())
			return tag.get().getYaw();
		return 0;
	}

	public static double getTargetPitch(int aprilTag) {
		Optional<PhotonTrackedTarget> tag = getTagTarget(aprilTag);
		if (tag.isPresent())
			return tag.get().getYaw();
		return 0;
	}

	public static double getDistanceToTarget(double targetHeight) {
		if (hasResults()) {
			return PhotonUtils.calculateDistanceToTargetMeters(VisionConstants.CAMERA_HEIGHT_METERS, targetHeight,
					VisionConstants.CAMERA_PITCH_RADIANS, getTargetPitch());
		}
		return -1;
	}

	public static double getDistanceToTarget(int aprilTag) {
		if (getTagTarget(aprilTag).isPresent()) {
			return PhotonUtils.calculateDistanceToTargetMeters(VisionConstants.CAMERA_HEIGHT_METERS,
					VisionConstants.aprilTagHeights[aprilTag], VisionConstants.CAMERA_PITCH_RADIANS,
					getTargetPitch(aprilTag));
		}
		return -1;
	}

	public static int getBestTargetID() {
		if (hasResults())
			return camera.getLatestResult().getBestTarget().getFiducialId();
		return -1;
	}

	public static Optional<PhotonTrackedTarget> getTagTarget(int tag) {
		if (hasResults())
			for (PhotonTrackedTarget target : camera.getLatestResult().getTargets())
				if (target.getFiducialId() == tag)
					return Optional.of(target);
		return Optional.empty();
	}

	public static Transform3d getPose() {
		PhotonPipelineResult result = camera.getLatestResult();
		if (result.getMultiTagResult().estimatedPose.isPresent) {
			return result.getMultiTagResult().estimatedPose.best;
		}
		return null;
	}
}
