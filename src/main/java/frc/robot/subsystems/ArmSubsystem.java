// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
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
  public double[] armHeights = {0.51,0.85,-4,30,0,0.8};

  public static double armP=0.1;
  public static double armI=0.0;
  public static double armD=0.0;
  public static double armFeed_Forward=0.0;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    armMotor = new CANSparkMax(Constants.DriveConstants.kArmCanId, CANSparkLowLevel.MotorType.kBrushless);
    armEncoder = armMotor.getAbsoluteEncoder(Type.kDutyCycle);
    armController = armMotor.getPIDController();
    armMotor.setSmartCurrentLimit(40);

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

  public void moveArm(double speed){
    armMotor.set(speed);
  }
  public Command doAutoAim(double distance, RobotTarget target){
    final double ANGULAR_P = 0.1;
    final double ANGULAR_D = 0.0;
    final double height;
    if (target == RobotTarget.SPEAKER){
      height = Constants.SPEAKER_HEIGHT_METERS;
    } else {
      height = Constants.ampHeightBottom;
    }
    final double distanceToTarget = Vision.distanceFromTarget(height);
    final double angleToTag = Vision.getTargetPitch(false);
    // armMotor.get
    final double angleCurrent = armMotor.getEncoder().getPosition(); // TODO this could be a wrong way to get angle, we need absoluteEncoder most likely
    final double angle = ShootingUtils.getOptimalAngle(distance, angleToTag, angleCurrent);
    
// ;
//     return Commands.runOnce(() -> 
//      armController.setReference() // TODO make it go to angle
//     );
      
}

}
