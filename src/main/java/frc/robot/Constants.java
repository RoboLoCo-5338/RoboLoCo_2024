// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    
    
  }

  public static final int RIGHTREAR_MOTOR = 1;
  public static final int RIGHTFRONT_MOTOR = 2;
  public static final int LEFTREAR_MOTOR = 3;
  public static final int LEFTFRONT_MOTOR = 4;
  // public static final int ELEVATOR_MOTOR = 7;
  public static final int CONE_TIPPER = 0;
  // public static final int EFFECTOR_MOTOR = 5;
  // public static final int ARM_MOTOR = 6;

  public static final int ABUTTON = 1; 
  public static final int BBUTTON = 2; 
  public static final int XBUTTON = 3; 
  public static final int YBUTTON = 4; 
  public static final int LBBUTTON = 5; 
  public static final int RBBUTTON = 6;
  public static final int BACKBUTTON = 7; 
  public static final int STARTBUTTON = 8; 
  public static final int LEFTSTICKBUTTON = 9; 
  public static final int RIGHTSTICKBUTTON = 10;

<<<<<<< HEAD

  public enum RobotTarget {
    SPEAKER,
    AMP
  }
  //field information
  public static final double ampHeightBottom = Units.feetToMeters(2) + Units.inchesToMeters(2); // in meters
  public static final double pocketHeight=Units.feetToMeters(1)+Units.inchesToMeters(6);
  public static final double ampHeightTop = ampHeightBottom+pocketHeight; 
  public static final double AMP_HEIGHT_CENTER = (ampHeightBottom + ampHeightTop)/2; 
  
  public static final double[] aprilTagHeights = { //These are to the bottom of the AprilTag
    Units.feetToMeters(4) + Units.inchesToMeters(1/8), //Blue Source
    Units.feetToMeters(4) + Units.inchesToMeters(1/8), //Blue Source
    Units.feetToMeters(4) + Units.inchesToMeters(3+7/8), //Red Speaker
    Units.feetToMeters(4) + Units.inchesToMeters(3+7/8), //Red Speaker
    Units.feetToMeters(4) + Units.inchesToMeters(1/8), //Red Amp
    Units.feetToMeters(4) + Units.inchesToMeters(1/8), //Blue Amp
    Units.feetToMeters(4) + Units.inchesToMeters(3+7/8), //Blue Speaker
    Units.feetToMeters(4) + Units.inchesToMeters(3+7/8), //Blue Speaker
    Units.feetToMeters(4) + Units.inchesToMeters(1/8), //Blue Source
    Units.feetToMeters(4) + Units.inchesToMeters(1/8), //Blue Source
    Units.feetToMeters(3) + Units.inchesToMeters(11.5), //Red Stage
    Units.feetToMeters(3) + Units.inchesToMeters(11.5), //Red Stage
    Units.feetToMeters(3) + Units.inchesToMeters(11.5), //Red Stage
    Units.feetToMeters(3) + Units.inchesToMeters(11.5), //Blue Stage
    Units.feetToMeters(3) + Units.inchesToMeters(11.5), //Blue Stage
    Units.feetToMeters(3) + Units.inchesToMeters(11.5), //Blue Stage
  };
  public static final double[] aprilTagUniqueHeights = { //These are to the bottom of the AprilTag
    Units.feetToMeters(4) + Units.inchesToMeters(1/8), //Source
    Units.feetToMeters(4) + Units.inchesToMeters(3+7/8), //Speaker
    Units.feetToMeters(4) + Units.inchesToMeters(1/8), //Amp
    Units.feetToMeters(3) + Units.inchesToMeters(11.5), //Stage
  };


  public static final double speakerHeightBottom = Units.feetToMeters(6) + Units.inchesToMeters(6);
  public static final double speakerHeightTop = Units.feetToMeters(6) + Units.inchesToMeters(10+7/8);
  public static final double plasticTagLength = Units.inchesToMeters(10.5);
  public static final double aluminumTagLength = Units.inchesToMeters(9);
  public static final double aprilTagLength = Units.inchesToMeters(8+(1/8));
  
  
  public static final double xOffsetFromCameraToPivot = 1.0; //TODO THIS IS NOT MEASURED YET
  public static final double yOffsetFromCameraToPivot = 1.0; //TODO THIS IS NOT MEASURED YET
  public static final double armLength = 1.0; //TODO ALSO NOT MEASURED
  // Constants such as camera and target height stored. Change per robot and goal!
  public static final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(42);
  public static final double SPEAKER_HEIGHT_METERS = Units.feetToMeters(2.66);
  // Angle between horizontal and the camera.
  public static final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);
  // How far from the target we want to be
  // public static final double GOAL_RANGE_METERS = Units.feetToMeters(3);
  
  public static final double speakerHeightAverage = (speakerHeightBottom + speakerHeightTop) / 2.0;

=======
>>>>>>> Arm
  // Height presets.
  public static final int Height = 3;
  public static final int height1 = 1;
  public static final int height2 = 2;
  public static final int height3 = 3;
  public static final int height4 = 4;
  public static final int height5 = 5;

  //speed_multi
  public static final double speed_multi = 0.1;
  public static final double kp = 0.01;
  public static final double min_command = 0.001;
<<<<<<< HEAD
=======

>>>>>>> Arm

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 1;
    public static final double kMaxAngularSpeed = Math.PI * 1; // radians per second
    public static final double kDirectionSlewRate = 10; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

  
    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(26.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(26.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));
    // public static final double speed_multi = 0.1;
    // public static final double kp = 0.01;
    // public static final double min_command = 0.001;
    
    // Angular offsets of the modules relative to the chassis in radians
    
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 6;
    public static final int kRearLeftDrivingCanId = 4;
    public static final int kFrontRightDrivingCanId = 8;
    public static final int kRearRightDrivingCanId = 2;

    public static final int kFrontLeftTurningCanId = 5;
    public static final int kRearLeftTurningCanId = 3;
    public static final int kFrontRightTurningCanId = 7;
    public static final int kRearRightTurningCanId = 1;

    // CHANGE CAN IDS of the Subsystems pls
    public static final int kArmCanId = -1;
    public static final int kShooter1CanId = -1;
    public static final int kShooter2CanId = -1;
    public static final int kIntakeIndexCanId = -1;

    public static final boolean kGyroReversed = true;

    public static HashMap<Double,String> armHashMap;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 13;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double kDriveDeadband = 0.10;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 1;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 0.01;
    public static final double kPYController = 0.01;
    public static final double kPThetaController = 1;



    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

   //public static final int HOOK_MOTOR = 9; // placeholder
  // public static final int EFFECTOR_MOTOR = 10;
  // public static final int INTAKE_MOTOR = 11;

}