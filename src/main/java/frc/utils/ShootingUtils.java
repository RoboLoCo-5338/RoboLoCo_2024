package frc.utils;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class ShootingUtils {

    private static final double DISTANCE_TO_PIVOT_X = 0.1524; // meters
    private static final double OFFSET_ANGLE_DEGREES = 40; // degrees
    private static final double ARM_LENGTH = 0.47752; // meters
    private static final double CAMERA_HEIGHT = Units.inchesToMeters(42); // meters
    private static final double SPEAKER_TAG_HEIGHT = 0.2667; // meters
    private static final double DISTANCE_TAG_TO_SPEAKER = Constants.SPEAKER_HEIGHT_METERS - SPEAKER_TAG_HEIGHT; // meters

    private static final double DISTANCE_CAMERA_TO_TAG_Y = SPEAKER_TAG_HEIGHT - CAMERA_HEIGHT;

    public static double getOptimalAngle(double distanceToTag, double angleToTag, double angleCurrent) {
        double resultDegree;
        double numerator = DISTANCE_CAMERA_TO_TAG_Y - (ARM_LENGTH * Math.sin( Math.toRadians(angleCurrent) )) + DISTANCE_TAG_TO_SPEAKER;
        double xDistance = distanceToTag*Math.cos(Math.toRadians(angleToTag));
        double denominator = xDistance + DISTANCE_TO_PIVOT_X + (ARM_LENGTH * Math.cos( Math.toRadians(angleCurrent) ));
        double fraction = numerator/denominator;
        double offsetRadians = Math.toRadians(OFFSET_ANGLE_DEGREES);
        resultDegree = offsetRadians - Math.atan(fraction);
        return Math.toDegrees(resultDegree);
    }

}
