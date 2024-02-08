package frc.robot.subsystems;
import java.io.ObjectInputStream.GetField;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import edu.wpi.first.math.util.Units;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import frc.robot.Constants.DriveConstants;;

public class Vision {
    static PhotonCamera camera = new PhotonCamera("Captain Rivets"); //Change the name when needed or things will break
    public Vision(){}
    public static boolean hasResults(){
        return camera.getLatestResult().hasTargets();
    }
    public static PhotonTrackedTarget getBestTarget(){
        return camera.getLatestResult().getBestTarget();
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
    public static double distanceFromTarget(){
        if(hasResults()){
            return PhotonUtils.calculateDistanceToTargetMeters(DriveConstants.CAMERA_HEIGHT_METERS, DriveConstants.TARGET_HEIGHT_METERS, DriveConstants.CAMERA_PITCH_RADIANS, getTargetPitch(true));
        }
        else{
            return Double.POSITIVE_INFINITY;
        }
    }
}
