// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {

  private CANSparkMax armMotor1;
  private CANSparkMax armMotor2;
  public SparkAbsoluteEncoder armEncoder1;
  public SparkPIDController armController1;
  public SparkAbsoluteEncoder armEncoder2;
  public SparkPIDController armController2;

  public static double armP = 0.1;
  public static double armI = 0.0;
  public static double armD = 0.0;
  public static double armFeed_Forward = 0.0;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    armMotor1 = new CANSparkMax(Constants.DriveConstants.kArmCanId1, MotorType.kBrushless);
    armMotor2 = new CANSparkMax(Constants.DriveConstants.kArmCanId2, MotorType.kBrushless);
    armEncoder1 = armMotor1.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    armController1 = armMotor1.getPIDController();
    armEncoder2 = armMotor2.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    armController2 = armMotor2.getPIDController();
    armMotor1.setSmartCurrentLimit(40);
    armMotor2.setSmartCurrentLimit(40);

    configController();
  }

  private void configController() {
    // PID config for the arm
    armController1.setP(armP);
    armController1.setI(armI);
    armController1.setD(armD);
    armController1.setFF(armFeed_Forward);
    armController2.setP(armP);
    armController2.setI(armI);
    armController2.setD(armD);
    armController2.setFF(armFeed_Forward);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // public void resetArm(){
  //   armEncoder.setPosition(0);
  // }

  public double getArmPosition() {
    return armEncoder1.getPosition();
  }

  public void stopArm() {
    armMotor1.set(0);
    armMotor2.set(0);
  }

  public void setArm(double position) {
    armController1.setReference(position, CANSparkMax.ControlType.kPosition);
    armController2.setReference(-position, CANSparkMax.ControlType.kPosition); // TODO This may not work check the position
 }
  public void moveArm(double speed) {
    armMotor1.set(speed);
    armMotor2.set(-speed);
  }
}