// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
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

public class IntakeSubsystem extends SubsystemBase {
 //this code is just copied from the ArmSubsystem
  private CANSparkBase intakeMotor;
  public SparkAbsoluteEncoder intakeEncoder;
  public SparkPIDController intakeController;


  public static double intakeP=0.1;
  public static double intakeI=0.0;
  public static double intakeD=0.0;
  public static double intake_Forward=0.0;
  // public static double GEAR_RATIO = 200;


  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    intakeMotor = new CANSparkMax(Constants.DriveConstants.kArmCanId, CANSparkLowLevel.MotorType.kBrushless);
    intakeEncoder = intakeMotor.getAbsoluteEncoder(Type.kDutyCycle);
    intakeController = intakeMotor.getPIDController();
    intakeMotor.setSmartCurrentLimit(40);

    configController();
  }

  private void configController(){
    // PID config for the arm
    intakeController.setP(intakeP);
    intakeController.setI(intakeI);
    intakeController.setD(intakeD);
    intakeController.setFF(intake_Forward);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // public void resetArm(){
  //   armEncoder.setPosition(0);
  // }


  public double getArmPosition(){
    return intakeEncoder.getPosition();
  }

  public void stopArm(){
    intakeMotor.set(0);
  }

  public void setArm(double position){
    intakeController.setReference(position, CANSparkMax.ControlType.kPosition); // the docs will say that this method is deprecated, but apparently the documentation is wrong
    //idfk anymore
  }
  public void intakeInward() {
    intakeMotor.set(0.5);
  }
  public void intakeOutward() {
    intakeMotor.set(-0.5);
  }
  public void moveArm(double speed){
    intakeMotor.set(speed);
  }
  public double getInitialAngle() {
    return -10; //this is a placeholder
  }
 

}
