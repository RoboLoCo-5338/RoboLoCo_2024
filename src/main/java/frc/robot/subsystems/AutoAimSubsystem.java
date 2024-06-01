package frc.robot.subsystems;


import frc.robot.Constants;
import frc.robot.RobotContainer;

public class AutoAimSubsystem {

    private static final double OFFSET_ANGLE_DEGREES = Math.toRadians(40); // radians
    private static final double SPEAKER_TAG_HEIGHT = Constants.aprilTagUniqueHeights[1]+Constants.plasticTagLength/2; // meters
    private static final double DISTANCE_TAG_TO_SPEAKER = Constants.SPEAKER_HEIGHT_METERS - SPEAKER_TAG_HEIGHT; // meters
    private static final double radians_per_rotation = 2 * Math.PI;
    private static final double rotations_per_radians = 1/radians_per_rotation;
    private static final double DISTANCE_CAMERA_TO_TAG_Y = SPEAKER_TAG_HEIGHT - Constants.CAMERA_HEIGHT_METERS;

    public static double getOptimalAngleRadians(double distanceToTag, double angleToTag, double angleCurrent) {
        double resultDegree;
        double numerator = DISTANCE_CAMERA_TO_TAG_Y - (Constants.ARM_LENGTH * Math.sin(angleCurrent)) + DISTANCE_TAG_TO_SPEAKER;
        double xDistance = distanceToTag*Math.cos(angleToTag);
        double denominator = xDistance + Constants.X_OFFSET_CAMERA_TO_PIVOT + (Constants.ARM_LENGTH * angleCurrent);
        double fraction = numerator/denominator;
        double offsetRadians = OFFSET_ANGLE_DEGREES;
        resultDegree = offsetRadians - Math.atan(fraction);
        return resultDegree;
    }
    public static double getOptimalAngleDegrees(double distanceToTag, double angleToTag, double angleCurrent) {
        double resultDegree;
        double numerator = DISTANCE_CAMERA_TO_TAG_Y - (Constants.ARM_LENGTH * Math.sin( Math.toRadians(angleCurrent) )) + DISTANCE_TAG_TO_SPEAKER;
        double xDistance = distanceToTag*Math.cos(Math.toRadians(angleToTag));
        double denominator = xDistance + Constants.X_OFFSET_CAMERA_TO_PIVOT + (Constants.ARM_LENGTH * Math.cos( Math.toRadians(angleCurrent) ));
        double fraction = numerator/denominator;
        double offsetRadians = OFFSET_ANGLE_DEGREES;
        resultDegree = offsetRadians - Math.atan(fraction);
        return Math.toDegrees(resultDegree);
    }
    public static double getCurrentAngle() {
        double rotations = RobotContainer.m_Arm.getArmPosition(); // only gets motor 1 but it doesnt matter
        double angle_bad_offset = rotations * radians_per_rotation;
        double angle_offset_good = angle_bad_offset + Constants.ARM_ANGLE_HORIZONTAL_OFFSET;
        return angle_offset_good;
    }
    /*
    public static void autoAim() {
        //red 4  blue 7
        Vision.turnToTagCommand();
        double angle = Vision.getTargetPitch(true);
        double distance = Vision.distanceFromTarget(Constants.SPEAKER_HEIGHT_METERS);// the height of the april tag, not hte target, which is good
        double currentAngle = getCurrentAngle();
        double optimalAngle = getOptimalAngleRadians(distance, angle, currentAngle);
        RobotContainer.m_Arm.setArmRadians(optimalAngle);
    }
    */
  

}