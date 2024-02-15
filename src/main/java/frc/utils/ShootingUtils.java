package frc.utils;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class ShootingUtils {

    private static final double OFFSET_ANGLE_DEGREES = 40; // degrees
    private static final double SPEAKER_TAG_HEIGHT = 0.2667; // meters
    private static final double DISTANCE_TAG_TO_SPEAKER = Constants.SPEAKER_HEIGHT_METERS - SPEAKER_TAG_HEIGHT; // meters

    private static final double DISTANCE_CAMERA_TO_TAG_Y = SPEAKER_TAG_HEIGHT - Constants.CAMERA_HEIGHT_METERS;

    public static double getOptimalAngle(double distanceToTag, double angleToTag, double angleCurrent) {
        double resultDegree;
        double numerator = DISTANCE_CAMERA_TO_TAG_Y - (Constants.armLength * Math.sin( Math.toRadians(angleCurrent) )) + DISTANCE_TAG_TO_SPEAKER;
        double xDistance = distanceToTag*Math.cos(Math.toRadians(angleToTag));
        double denominator = xDistance + Constants.xOffsetFromCameraToPivot + (Constants.armLength * Math.cos( Math.toRadians(angleCurrent) ));
        double fraction = numerator/denominator;
        double offsetRadians = Math.toRadians(OFFSET_ANGLE_DEGREES);
        resultDegree = offsetRadians - Math.atan(fraction);
        return Math.toDegrees(resultDegree);
    }

}
