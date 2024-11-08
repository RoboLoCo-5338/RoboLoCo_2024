package frc.robot.subsystems;

 /* import java.util.Optional;
 *
 * import org.photonvision.PhotonCamera;
 * import org.photonvision.PhotonPoseEstimator;
 * import org.photonvision.PhotonPoseEstimator.PoseStrategy;
 * import org.photonvision.PhotonUtils;
 * 
 * import edu.wpi.first.apriltag.AprilTagFieldLayout;
 * import edu.wpi.first.apriltag.AprilTagFields;
 * import edu.wpi.first.math.MathUtil;
 * import edu.wpi.first.math.controller.PIDController;
 * import edu.wpi.first.math.geometry.Pose3d;
 * import edu.wpi.first.math.geometry.Rotation3d;
 * import edu.wpi.first.math.geometry.Transform3d;
 * import edu.wpi.first.math.geometry.Translation3d;
 * import edu.wpi.first.math.util.Units;
 * import edu.wpi.first.wpilibj2.command.Command;
 * import edu.wpi.first.wpilibj2.command.Commands;
 * import edu.wpi.first.cameraserver.CameraServer;
 * 
 * import org.photonvision.targeting.PhotonTrackedTarget;
 * 
 * import frc.robot.Constants;
 * import frc.robot.Robot;
 * import frc.robot.RobotContainer;
 * // Speaker id's are 4 (red) 7 (blue)
 * import frc.robot.Constants.OIConstants;
 * public class Vision {
 * static final double ANGULAR_P = 0.1;
 * static final double ANGULAR_D = 0.0;
 * static AprilTagFieldLayout aprilTagFieldLayout =
 * AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(); // this is the
 * coolest method ever
 * static PhotonCamera camera = new PhotonCamera("Rock'n Rivets"); //Change the
 * name when needed or things will break
 * // notes about robotToCam
 * // the Translation3d is from the CENTER of the robot
 * //
 * https://drive.google.com/file/d/14ehBwUCPgZg1t8bk1RZi0HZHaisDKb0k/view?usp=
 * sharing very professional diagram
 * static Transform3d robotToCam = new Transform3d(new
 * Translation3d(Constants.X_OFFSET_CAMERA_TO_PIVOT,
 * Constants.Y_OFFSET_CAMERA_TO_PIVOT, Constants.Z_OFFSET_CAMERA_TO_PIVOT), new
 * Rotation3d(0,0,0));
 * static PhotonPoseEstimator photonPoseEstimator = new
 * PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.LOWEST_AMBIGUITY ,
 * camera, robotToCam);
 * // Pose3d robotPose =
 * PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(),
 * aprilTagFieldLayout.getTagPose(target.getFiducialId()), cameraToRobot);
 * //TODO change PoseStrategy from LOWEST_AMBIGUITY to
 * MULTI_TAG_PNP_ON_COPROCESSOR
 * //https://docs.photonvision.org/en/latest/docs/programming/photonlib/robot-
 * pose-estimator.html
 * // apparently it improves performance but needs some setup and im not about
 * doing setup rn
 * private static PIDController turnController=new PIDController(ANGULAR_P, 0,
 * ANGULAR_D);
 * 
 * public Vision(){}
 * public static boolean hasResults(){
 * return camera.getLatestResult().hasTargets();
 * }
 * public static PhotonTrackedTarget getBestTarget(){
 * return camera.getLatestResult().getBestTarget();
 * }
 * public static Optional<Pose3d> getPoseRelativeToAprilTag() {
 * if (hasResults()) { //im trying to find a way to get the 3d pose difference
 * between the speaker april tag and the robot. the docs say
 * // its possible but ITS SO FRICKING VAGUE AHHHH
 * PhotonTrackedTarget target = camera.getLatestResult().getBestTarget();
 * if (target.getFiducialId() == 7 || target.getFiducialId() == 4) {
 * Pose3d tagPose =
 * aprilTagFieldLayout.getTagPose(target.getFiducialId()).get();
 * Pose3d robotPose =
 * PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(),
 * tagPose, robotToCam);
 * Pose3d difference = tagPose.relativeTo(robotPose);
 * return Optional.of(difference);
 * }
 * }
 * return Optional.empty();
 * 
 * }
 * public static Command turnToTagCommand(){ //untested
 * if(!Vision.hasResults()){return null;}
 * else{
 * return Commands.sequence(
 * RobotContainer.m_robotDrive.runOnce(() -> RobotContainer.m_robotDrive.drive(
 * -MathUtil.applyDeadband(RobotContainer.m_driverController.getLeftY(),
 * OIConstants.kDriveDeadband),
 * -MathUtil.applyDeadband(RobotContainer.m_driverController.getLeftX(),
 * OIConstants.kDriveDeadband),
 * -MathUtil.applyDeadband(turnController.calculate(Vision.getTargetYaw(false)),
 * OIConstants.kDriveDeadband),
 * true, true,false)
 * )
 * );
 * }
 * }
 * public static double getTargetPitch(boolean radians){ //The boolean is if you
 * want or don't want radians
 * if(hasResults()){
 * double output=getBestTarget().getPitch();
 * if(radians){
 * output=Units.degreesToRadians(output);
 * }
 * return output;
 * }
 * return Double.POSITIVE_INFINITY;
 * }public static double getTargetYaw(boolean radians){ //The boolean is if you
 * want or don't want radians
 * if(hasResults()){
 * double output=getBestTarget().getYaw();
 * if(radians){
 * output=Units.degreesToRadians(output);
 * }
 * return output;
 * }
 * return Double.POSITIVE_INFINITY;
 * }
 * public static double distanceFromTarget(double targetHeight){
 * if(hasResults()){
 * return
 * PhotonUtils.calculateDistanceToTargetMeters(Constants.CAMERA_HEIGHT_METERS,
 * targetHeight, Constants.CAMERA_PITCH_RADIANS, getTargetPitch(true));
 * }
 * else{
 * return Double.POSITIVE_INFINITY;
 * }
 * }
 * }
 */
