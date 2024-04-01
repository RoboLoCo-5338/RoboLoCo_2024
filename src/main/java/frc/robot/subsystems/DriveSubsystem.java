// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotContainer;
import frc.utils.SwerveUtils;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  public final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  public final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  public final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  public final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  public final Pigeon2 m_gyro = new Pigeon2(0);

  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {

    AutoBuilder.configureHolonomic(
      this::getPose,
      this::resetOdometry,
      this::getRobotRelativeSpeeds,
      this::driveRobotRelative,
      new HolonomicPathFollowerConfig(
        new PIDConstants(5.0, 0.0, 0.0), 
        new PIDConstants(5.0, 0.0, 0.0), 
        Constants.AutoConstants.kMaxSpeedMetersPerSecond, 
        0.394, 
        new ReplanningConfig()),
    ()->{
         var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
 }
    

  

  public void driveRobotRelative(ChassisSpeeds chassisSpeeds){
    drive(chassisSpeeds.vxMetersPerSecond,chassisSpeeds.vyMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond, false, true);
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
    
    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
      }
      

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45*Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      }
      else if (angleDif > 0.85*Math.PI) {
        if (m_currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        }
        else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      }
      else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;
      
      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rot);


    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond*(RobotContainer.slowMode?0.6:1.0);
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond*(RobotContainer.slowMode?0.6:1.0);
    double rotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed*(RobotContainer.slowMode?0.6:1.0);

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble()* (DriveConstants.kGyroReversed ? -1.0 : 1.0)))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  public ChassisSpeeds getRobotRelativeSpeeds(){
    return DriveConstants.kDriveKinematics.toChassisSpeeds(
      m_frontLeft.getState(),
      m_frontRight.getState(),
      m_rearLeft.getState(),
      m_rearRight.getState());
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble()).getDegrees()* (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }



  //SYSID CODE 

  public void setVoltageDrive(double volts){
    m_frontLeft.m_drivingSparkMax.setVoltage(volts);
    m_frontRight.m_drivingSparkMax.setVoltage(-volts);
    m_rearLeft.m_drivingSparkMax.setVoltage(volts);
    m_rearRight.m_drivingSparkMax.setVoltage(-volts);
  }

   public void setVoltageSteer(double volts){
    m_frontLeft.m_turningSparkMax.setVoltage(volts);
    m_frontRight.m_turningSparkMax.setVoltage(volts);
    m_rearLeft.m_turningSparkMax.setVoltage(volts);
    m_rearRight.m_turningSparkMax.setVoltage(volts);
  }

  private final MutableMeasure<Voltage> m_appliedVoltage = MutableMeasure.mutable(Units.Volts.of(0));
  private final MutableMeasure<Distance> m_distance = MutableMeasure.mutable(Units.Meters.of(0));
  private final MutableMeasure<Velocity<Distance>> m_velocity = MutableMeasure.mutable(Units.MetersPerSecond.of(0));

  public SysIdRoutine m_driveSysIdRoutine =
    new SysIdRoutine(
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(
        (Measure<Voltage> volts) -> setVoltageDrive(volts.in(Units.Volts)),
         
        log -> {

          // log motor information, someone please check all the units and their conversions, I'm not quite sure if I did that part right
          log.motor("drive-front-left")
            .voltage(
              m_appliedVoltage.mut_replace(
                m_frontLeft.m_drivingSparkMax.getBusVoltage(),Units.Volts
              )
            )
            .linearPosition(m_distance.mut_replace(
              m_frontLeft.m_drivingEncoder.getPosition()*m_frontLeft.m_drivingEncoder.getPositionConversionFactor(),Units.Meters))
            .linearVelocity(
              m_velocity.mut_replace(
                m_frontLeft.m_drivingEncoder.getVelocity()*m_frontLeft.m_drivingEncoder.getVelocityConversionFactor(),Units.MetersPerSecond
            ));

           log.motor("drive-front-right")
            .voltage(
              m_appliedVoltage.mut_replace(
                m_frontRight.m_drivingSparkMax.getBusVoltage(),Units.Volts
              )
            )
            .linearPosition(m_distance.mut_replace(
              m_frontRight.m_drivingEncoder.getPosition()*m_frontRight.m_drivingEncoder.getPositionConversionFactor(),Units.Meters))
            .linearVelocity(
              m_velocity.mut_replace(
                m_frontRight.m_drivingEncoder.getVelocity()*m_frontRight.m_drivingEncoder.getVelocityConversionFactor(),Units.MetersPerSecond
            ));

             log.motor("drive-rear-left")
            .voltage(
              m_appliedVoltage.mut_replace(
                m_rearLeft.m_drivingSparkMax.getBusVoltage(),Units.Volts
              )
            )
            .linearPosition(m_distance.mut_replace(
              m_rearLeft.m_drivingEncoder.getPosition()*m_rearLeft.m_drivingEncoder.getPositionConversionFactor(),Units.Meters))
            .linearVelocity(
              m_velocity.mut_replace(
                m_rearLeft.m_drivingEncoder.getVelocity()*m_rearLeft.m_drivingEncoder.getVelocityConversionFactor(),Units.MetersPerSecond
            ));

             log.motor("drive-rear-right")
            .voltage(
              m_appliedVoltage.mut_replace(
                m_rearRight.m_drivingSparkMax.getBusVoltage(),Units.Volts
              )
            )
            .linearPosition(m_distance.mut_replace(
              m_rearRight.m_drivingEncoder.getPosition()*m_rearRight.m_drivingEncoder.getPositionConversionFactor(),Units.Meters))
            .linearVelocity(
              m_velocity.mut_replace(
                m_rearRight.m_drivingEncoder.getVelocity()*m_rearRight.m_drivingEncoder.getVelocityConversionFactor(),Units.MetersPerSecond
            ));
        }, 
         this));

  public SysIdRoutine m_steerSysIdRoutine = 
    new SysIdRoutine(
      new SysIdRoutine.Config(),  
      new SysIdRoutine.Mechanism(
        (Measure<Voltage> volts) -> setVoltageSteer(volts.in(Units.Volts)),
       log -> {
         log.motor("turn-front-left")
            .voltage(
              m_appliedVoltage.mut_replace(
                m_frontLeft.m_turningSparkMax.getBusVoltage(),Units.Volts
              )
            )
            .linearPosition(m_distance.mut_replace(
              m_frontLeft.m_turningEncoder.getPosition()*m_frontLeft.m_turningEncoder.getPositionConversionFactor(),Units.Meters))
            .linearVelocity(
              m_velocity.mut_replace(
                m_frontLeft.m_turningEncoder.getVelocity()*m_frontLeft.m_turningEncoder.getVelocityConversionFactor(),Units.MetersPerSecond
            ));

          log.motor("turn-front-right")
            .voltage(
              m_appliedVoltage.mut_replace(
                m_frontRight.m_turningSparkMax.getBusVoltage(),Units.Volts
              )
            )
            .linearPosition(m_distance.mut_replace(
              m_frontRight.m_turningEncoder.getPosition()*m_frontRight.m_turningEncoder.getPositionConversionFactor(),Units.Meters))
            .linearVelocity(
              m_velocity.mut_replace(
                m_frontRight.m_turningEncoder.getVelocity()*m_frontRight.m_turningEncoder.getVelocityConversionFactor(),Units.MetersPerSecond
            ));

             log.motor("drive-rear-left")
            .voltage(
              m_appliedVoltage.mut_replace(
                m_rearLeft.m_turningSparkMax.getBusVoltage(),Units.Volts
              )
            )
            .linearPosition(m_distance.mut_replace(
              m_rearLeft.m_turningEncoder.getPosition()*m_rearLeft.m_turningEncoder.getPositionConversionFactor(),Units.Meters))
            .linearVelocity(
              m_velocity.mut_replace(
                m_rearLeft.m_turningEncoder.getVelocity()*m_rearLeft.m_turningEncoder.getVelocityConversionFactor(),Units.MetersPerSecond
            ));

             log.motor("drive-rear-right")
            .voltage(
              m_appliedVoltage.mut_replace(
                m_rearRight.m_turningSparkMax.getBusVoltage(),Units.Volts
              )
            )
            .linearPosition(m_distance.mut_replace(
              m_rearRight.m_turningEncoder.getPosition()*m_rearRight.m_turningEncoder.getPositionConversionFactor(),Units.Meters))
            .linearVelocity(
              m_velocity.mut_replace(
                m_rearRight.m_turningEncoder.getVelocity()*m_rearRight.m_turningEncoder.getVelocityConversionFactor(),Units.MetersPerSecond
            ));
       }, 
        this));

  // someone figure out what this routine does, I'm not qute sure
  public SysIdRoutine m_slipSysIdRoutine=
  new SysIdRoutine(
    new SysIdRoutine.Config(Units.Volts.of(0.25).per(Units.Seconds.of(1)),null,null,null),
    new SysIdRoutine.Mechanism(
      (Measure<Voltage> volts) -> setVoltageDrive(volts.in(Units.Volts)),
        log -> {

          // log motor information, someone please check all the units and their conversions, I'm not quite sure if I did that part right
          log.motor("drive-front-left")
            .voltage(
              m_appliedVoltage.mut_replace(
                m_frontLeft.m_drivingSparkMax.getBusVoltage(),Units.Volts
              )
            )
            .linearPosition(m_distance.mut_replace(
              m_frontLeft.m_drivingEncoder.getPosition()*m_frontLeft.m_drivingEncoder.getPositionConversionFactor(),Units.Meters))
            .linearVelocity(
              m_velocity.mut_replace(
                m_frontLeft.m_drivingEncoder.getVelocity()*m_frontLeft.m_drivingEncoder.getVelocityConversionFactor(),Units.MetersPerSecond
            ));

           log.motor("drive-front-right")
            .voltage(
              m_appliedVoltage.mut_replace(
                m_frontRight.m_drivingSparkMax.getBusVoltage(),Units.Volts
              )
            )
            .linearPosition(m_distance.mut_replace(
              m_frontRight.m_drivingEncoder.getPosition()*m_frontRight.m_drivingEncoder.getPositionConversionFactor(),Units.Meters))
            .linearVelocity(
              m_velocity.mut_replace(
                m_frontRight.m_drivingEncoder.getVelocity()*m_frontRight.m_drivingEncoder.getVelocityConversionFactor(),Units.MetersPerSecond
            ));

             log.motor("drive-rear-left")
            .voltage(
              m_appliedVoltage.mut_replace(
                m_rearLeft.m_drivingSparkMax.getBusVoltage(),Units.Volts
              )
            )
            .linearPosition(m_distance.mut_replace(
              m_rearLeft.m_drivingEncoder.getPosition()*m_rearLeft.m_drivingEncoder.getPositionConversionFactor(),Units.Meters))
            .linearVelocity(
              m_velocity.mut_replace(
                m_rearLeft.m_drivingEncoder.getVelocity()*m_rearLeft.m_drivingEncoder.getVelocityConversionFactor(),Units.MetersPerSecond
            ));

             log.motor("drive-rear-right")
            .voltage(
              m_appliedVoltage.mut_replace(
                m_rearRight.m_drivingSparkMax.getBusVoltage(),Units.Volts
              )
            )
            .linearPosition(m_distance.mut_replace(
              m_rearRight.m_drivingEncoder.getPosition()*m_rearRight.m_drivingEncoder.getPositionConversionFactor(),Units.Meters))
            .linearVelocity(
              m_velocity.mut_replace(
                m_rearRight.m_drivingEncoder.getVelocity()*m_rearRight.m_drivingEncoder.getVelocityConversionFactor(),Units.MetersPerSecond
            ));
        },
       this));

  public Command drive_sysIdQuasistatic(SysIdRoutine.Direction direction){
    return m_driveSysIdRoutine.quasistatic(direction);
  }

  public Command steer_sysIdQuasistatic(SysIdRoutine.Direction direction){
    return m_steerSysIdRoutine.quasistatic(direction);
  }

  public Command driveSlipTestSysId (SysIdRoutine.Direction direction){
    return m_slipSysIdRoutine.quasistatic(direction);
  }

  public Command drive_sysIdDynamic(SysIdRoutine.Direction direction){
    return m_driveSysIdRoutine.dynamic(direction);
  }

  public Command steer_sysIdDynamic(SysIdRoutine.Direction direction){
    return m_steerSysIdRoutine.dynamic(direction);
  }

}
