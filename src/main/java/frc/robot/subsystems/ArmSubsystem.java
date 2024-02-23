// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.RobotTarget;
import frc.utils.ShootingUtils;
// import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {

  private CANSparkBase armMotor;
  public SparkAbsoluteEncoder armEncoder;
  public SparkPIDController armController;

  private CANSparkBase feedMotor;
  public SparkAbsoluteEncoder feedEncoder;
  public SparkPIDController feedController;

  private CANSparkBase shootMotor;
  public SparkAbsoluteEncoder shootEncoder;
  public SparkPIDController shootController;


  public static double armP=0.1;
  public static double armI=0.0;
  public static double armD=0.0;
  public static double armFeed_Forward=0.0;
  public static double angle_to_rotations = 0.159155;
  public static double rotations_to_angle = 6.2832;
  public static double GEAR_RATIO = 200;


  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    armMotor = new CANSparkMax(Constants.DriveConstants.kArmCanId, CANSparkLowLevel.MotorType.kBrushless);
    armEncoder = armMotor.getAbsoluteEncoder(Type.kDutyCycle);
    armController = armMotor.getPIDController();
    armMotor.setSmartCurrentLimit(40);
    
    feedMotor = new CANSparkMax(Constants.DriveConstants.kArmCanId, CANSparkLowLevel.MotorType.kBrushless);
    feedEncoder = feedMotor.getAbsoluteEncoder(Type.kDutyCycle);
    feedController = armMotor.getPIDController();
    feedMotor.setSmartCurrentLimit(40);

    shootMotor = new CANSparkMax(Constants.DriveConstants.kArmCanId, CANSparkLowLevel.MotorType.kBrushless);
    shootMotor.enableSoftLimit(SoftLimitDirection.kReverse, true); // TODO it could also be kForward
    shootEncoder = armMotor.getAbsoluteEncoder(Type.kDutyCycle);
    shootController = armMotor.getPIDController();
    shootMotor.setSmartCurrentLimit(40);

    configController();
  }

  private void configController(){
    // PID config for the arm
    armController.setP(armP);
    armController.setI(armI);
    armController.setD(armD);
    armController.setFF(armFeed_Forward);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // public void resetArm(){
  //   armEncoder.setPosition(0);
  // }

  public double getArmPosition(){
    return armEncoder.getPosition();
  }

  public void stopArm(){
    armMotor.set(0);
  }

  public void setArm(double position){
    armController.setReference(position, CANSparkMax.ControlType.kPosition); // the docs will say that this method is deprecated, but apparently the documentation is wrong
    //idfk anymore
  }

  public void shootOut() {
    feedMotor.set(0.5);
    shootMotor.set(0.5);
  }
  public void shootIn() {
    feedMotor.set(-0.5);
    shootMotor.set(-0.5);
  }
  public void moveArm(double speed){
    armMotor.set(speed);
  }
  public double getInitialAngle() {
    return 0; //this is a placeholder
  }
  public Command doAutoAim(RobotTarget target){


    // POSSIBLE PROBLEMS
    // angle is being measured wrong, idk how getPitch() works. It needs to be measured from the horizontal for this to work, which im pretty sure getPitch isn't doing
    // we have placeholder values rn
    // im not sure if getAbosluteEncoder is doing things right
    


    final double height;
    final RelativeEncoder m_encoder = armMotor.getEncoder();// TODO this could be a wrong way to get angle, idk if its getAbsoluteEncoder or getEncoder
    if (target == RobotTarget.SPEAKER){
      height = Constants.SPEAKER_HEIGHT_METERS;
    } else {
      height = Constants.AMP_HEIGHT_CENTER; // although im not sure if we're using autoaim for amp so this should never happen
    }
    final double distanceToTarget = Vision.distanceFromTarget(height);
    final double angleToTag = Vision.getTargetPitch(true);
    // armMotor.get
    final double rotationsCurrent = m_encoder.getPosition(); 
    final double angleCurrent = rotationsCurrent * rotations_to_angle;
    final double angle = ShootingUtils.getOptimalAngleRadians(distanceToTarget, angleToTag, angleCurrent);
    

    // final double rotations = 0.5;
    armMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) (Math.toRadians(135)*rotations_to_angle*GEAR_RATIO)); //SOFT LIMIT
    return Commands.sequence(
    Commands.runOnce(() -> 
     armController.setReference(getAngle(angle), CANSparkMax.ControlType.kPosition)
     
    ));
  
}
  public double getAngle(double rotations) {

      return (rotations)*GEAR_RATIO;
  }
    
}
