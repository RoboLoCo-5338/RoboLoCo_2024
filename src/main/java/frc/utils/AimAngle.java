package frc.utils;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.LinkedList;
import java.util.List;

public class AimAngle {
  private static final float GRAVITY = 9.8f;

  private static final double HEIGHT = 0.1;

  private static final double DX = 0.1;
  private static final double DY = 0.1;

  public static void main(String[] args) throws IOException {
    System.out.println("Speaker Height: " + HEIGHT);
    System.out.println("Gravity: " + GRAVITY);
    System.out.println("Distance Precision: " + DX);
    System.out.println("Velocity Precision: " + DY);

    LinkedList<String> distances = new LinkedList<String>();
    LinkedList<String> velocities = new LinkedList<String>();
    LinkedList<String> angles = new LinkedList<String>();

    // System.out.println("Distance\t\tVelocity\tAngle");
    for (double x = 0; x < 100; x += DX) {
      for (double v = 0; v < 20; v += DY) {
        double optimalAngle = getOptimalAngle(v, x, HEIGHT);

        if (optimalAngle > 0) {
          // System.out.println(formatDouble(x)+"\t\t\t"+formatDouble(v)+"\t\t"+formatDouble(optimalAngle));
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

    for(int i = 0; i < distances.size(); i++) {
      br.newLine();
      br.write(distances.get(i) + "," + velocities.get(i) + "," + angles.get(i));
    }

    br.close();
    System.out.println("Done!");
  }

  private static String formatDouble(double d) {
    return String.format("%.4f", d);
  }

  private static double getOptimalAngle(double shooterVelocity, double xDistance, double yHeight) {
    double velocitySquared = Math.pow(shooterVelocity, 2);
    double xDistanceSquared = Math.pow(xDistance, 2);
    double stub = (2 * velocitySquared) / (GRAVITY * xDistance);

    double calc1 = Math.pow(stub, 2);
    double calc2 = 4 * (((2 * velocitySquared) / (GRAVITY * xDistanceSquared) * yHeight) + 1);
    double calc3 = Math.sqrt(calc1 - calc2);
    double calc4 = stub - calc3;
    double calc5 = calc4 / 2;

    return Math.toDegrees(Math.atan(calc5));
  }
}
