package frc.utils;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Vision;

public class ShootingUtils {

    private static final double OFFSET_ANGLE_DEGREES = Math.toRadians(40); // radians
    private static final double SPEAKER_TAG_HEIGHT = Constants.aprilTagUniqueHeights[2]+Constants.plasticTagLength/2; // meters
    private static final double DISTANCE_TAG_TO_SPEAKER = Constants.SPEAKER_HEIGHT_METERS - SPEAKER_TAG_HEIGHT; // meters

    private static final double DISTANCE_CAMERA_TO_TAG_Y = SPEAKER_TAG_HEIGHT - Constants.CAMERA_HEIGHT_METERS;

    public static double getOptimalAngleRadians(double distanceToTag, double angleToTag, double angleCurrent) {
        double resultDegree;
        double numerator = DISTANCE_CAMERA_TO_TAG_Y - (Constants.armLength * Math.sin(angleCurrent)) + DISTANCE_TAG_TO_SPEAKER;
        double xDistance = distanceToTag*Math.cos(angleToTag);
        double denominator = xDistance + Constants.xOffsetFromCameraToPivot + (Constants.armLength * angleCurrent);
        double fraction = numerator/denominator;
        double offsetRadians = OFFSET_ANGLE_DEGREES;
        resultDegree = offsetRadians - Math.atan(fraction);
        return resultDegree;
    }
    public static double getOptimalAngleDegrees(double distanceToTag, double angleToTag, double angleCurrent) {
        double resultDegree;
        double numerator = DISTANCE_CAMERA_TO_TAG_Y - (Constants.armLength * Math.sin( Math.toRadians(angleCurrent) )) + DISTANCE_TAG_TO_SPEAKER;
        double xDistance = distanceToTag*Math.cos(Math.toRadians(angleToTag));
        double denominator = xDistance + Constants.xOffsetFromCameraToPivot + (Constants.armLength * Math.cos( Math.toRadians(angleCurrent) ));
        double fraction = numerator/denominator;
        double offsetRadians = OFFSET_ANGLE_DEGREES;
        resultDegree = offsetRadians - Math.atan(fraction);
        return Math.toDegrees(resultDegree);
    }
  

}
