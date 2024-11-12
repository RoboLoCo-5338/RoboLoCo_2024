// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
// import com.revrobotics.SparkPIDController; used for shootController 1/2 (currently unused)
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  private CANSparkMax shootMotor1;
  private CANSparkMax shootMotor2;
  private RelativeEncoder shootEncoder1;
  private RelativeEncoder shootEncoder2;

  // private SparkPIDController shootController1; idk what this is
  // private SparkPIDController shootController2; idk what this is

  public ShooterSubsystem() {
    shootMotor1 = new CANSparkMax(Constants.DriveConstants.kShooter1CanId, MotorType.kBrushless);
    shootMotor2 = new CANSparkMax(Constants.DriveConstants.kShooter2CanId, MotorType.kBrushless);
    shootEncoder1 = shootMotor1.getEncoder();
    shootEncoder2 = shootMotor2.getEncoder();
    // shootController1 = shootMotor1.getPIDController(); idk what this is
    // shootController2 = shootMotor2.getPIDController(); idk what this is
  }

  public void shooterForward() {

    // changed for inspire loudoun
    shootMotor1.set(0.9);
    shootMotor2.set(0.9);
  }

  public void shooterReverse() {
    shootMotor1.set(-0.3);
    shootMotor2.set(-0.3);
  }

  public void shooterReverseSlow() {
    shootMotor1.set(-0.1);
    shootMotor2.set(-0.1);
  }

  public void shooterStop() {
    shootMotor1.set(0);
    shootMotor2.set(0);
  }

  public double getEncoderPosition() {
    return (Math.abs(shootEncoder1.getPosition()) + Math.abs(shootEncoder2.getPosition()) / 2);
  }
}
