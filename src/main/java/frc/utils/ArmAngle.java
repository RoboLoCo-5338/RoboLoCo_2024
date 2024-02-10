package frc.utils;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.LinkedList;
import java.util.List;

import frc.robot.Constants;

public class AimAngle {

    public static void main(String[] args) throws IOException {
        double height = 7.8;
        double gravity = 9.8;
        double dx = 0.1;
        double dv = 0.1;


        System.out.println("Speaker Height: " + height);
        System.out.println("Gravity: " + gravity);
        System.out.println("Distance Precision: " + dx);
        System.out.println("Velocity Precision: " + dv);

        List<String> distances = new LinkedList<>();
        List<String> velocities = new LinkedList<>();
        List<String> angles = new LinkedList<>();

//        System.out.println("Distance\t\tVelocity\tAngle");
        for (double x = 0; x < 100; x += dx) {
            for (double v = 0; v < 20; v+= dv) {
                double optimalAngle = getOptimalAngle(v, gravity, x, height);
                if (optimalAngle > 0) {
//                    System.out.println(formatDouble(x)+"\t\t\t"+formatDouble(v)+"\t\t"+formatDouble(optimalAngle));
                    distances.add(formatDouble(x));
                    velocities.add(formatDouble(v));
                    angles.add(formatDouble(optimalAngle));
                }
            }
        }
        System.out.println("\n\nWriting to file...");
        File outputFile = new File("C:\\projects\\aim-logic\\calculations.csv");
        outputFile.delete();
        outputFile.createNewFile();
        BufferedWriter br = new BufferedWriter(new FileWriter(outputFile));
        br.write("Distance,Velocity,Angle");
        for(int i = 0; i<distances.size(); i++){
            br.newLine();
            br.write(distances.get(i) + "," + velocities.get(i) + "," + angles.get(i));
        }
        System.out.println("Done!");
    }

    private static String formatDouble(double d) {
        return String.format("%.4f", d);
    }

    // private static double getOptimalAngle(double shooterVelocity, double xDistance, double yHeight) {
    //     // WARNING: FOR THIS FUNCTION TO WORK, THE X DISTANCE AND Y DISTANCE MUST BE RELATIVE TO THE NOTE
    //     // YOU SHOULD PROBABLY USE THE ANGLE BASED ONE BECAUSE THAT ONE JUST TAKES MEASUREMENTS RELATIVE TO THE CAMERA
    //     //theres really zero point to this function
    //     float gravity = 9.8f;
    //     double velocitySquared = Math.pow(shooterVelocity, 2);
    //     double xDistanceSquared = Math.pow(xDistance, 2);
    //     double stub = (2 * velocitySquared) / (gravity * xDistance);
    //     double calc1 = Math.pow(stub,2);
    //     double calc2 = 4*( (((2*velocitySquared)/(gravity*xDistanceSquared))*yHeight) + 1 );
    //     double calc3 = Math.sqrt(calc1 - calc2);
    //     double calc4 = stub - calc3;
    //     double calc5 = calc4/2;
    //     double result = Math.atan(calc5);
    //     return Math.toDegrees(result);
    // }
    private static double getOptimalAngleWithAngle(double shooterVelocity, double distance, double theta, float heightOfTarget) {

        float gravity = 9.8f;
        double p_dx = Constants.armLength * Math.sin(theta);
        double p_dy = Constants.armLength * Math.cos(theta);
        double note_position_x = Constants.xOffsetFromCameraToPivot + p_dx;
        double note_position_y = Constants.yOffsetFromCameraToPivot + p_dy;
        double yHeight = heightOfTarget - note_position_y;
        // double xDistance = yHeight/Math.tan(theta); //this should not work
        
        double velocitySquared = Math.pow(shooterVelocity, 2);
        double xDistanceSquared = Math.pow(xDistance, 2);
        double stub = (2 * velocitySquared) / (gravity * xDistance);
        double calc1 = Math.pow(stub,2);
        double calc2 = 4*( (((2*velocitySquared)/(gravity*xDistanceSquared))*yHeight) + 1 );
        double calc3 = Math.sqrt(calc1 - calc2);
        double calc4 = stub - calc3;
        double calc5 = calc4/2;
        double result = Math.atan(calc5);
        return Math.toDegrees(result);
    }


}
