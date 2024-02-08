// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.reflect.Type;

import javax.swing.text.Position;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {

  private CANSparkMax armMotor;
  public SparkAbsoluteEncoder armEncoder;
  public SparkPIDController armController;
  public double[] armHeights = {0.51,0.85,-4,30,0,0.8};

  public static double armP=0.1;
  public static double armI=0.0;
  public static double armD=0.0;
  public static double armFeed_Forward=0.0;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    armMotor = new CANSparkMax(Constants.DriveConstants.kArmCanId, MotorType.kBrushless);
    armEncoder = armMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
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
    armController.setReference(position, CANSparkMax.ControlType.kPosition);
  }

  public void moveArm(double speed){
    armMotor.set(speed);
  }


}
