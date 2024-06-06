package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.ConfigurationFailedException;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class IntakeSubsystem extends SubsystemBase {
  private final CANSparkMax intakeMotor;
  private final CANSparkMax indexerMotor;
  public RelativeEncoder intakeEncoder;
  public RelativeEncoder indexerEncoder;
  private final LaserCan laserCan;

  public IntakeSubsystem() {
    laserCan = new LaserCan(DriveConstants.kLaserCanID);

    try {
      laserCan.setRangingMode(LaserCan.RangingMode.SHORT);
      laserCan.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
      laserCan.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    }
    catch (ConfigurationFailedException e) {
      System.out.println("Configuration failed! " + e);
    }

    intakeMotor = new CANSparkMax(DriveConstants.kIntakeMotorCanId /*Change Device ID*/, MotorType.kBrushless);
    indexerMotor = new CANSparkMax(DriveConstants.kIntakeIndexCanId /*Change Device ID*/, MotorType.kBrushless);

    indexerMotor.setIdleMode(IdleMode.kBrake);
    intakeEncoder = intakeMotor.getEncoder();
  }

  public void stopIntake() {
    intakeMotor.set(0);
  }

  public void intakeOut() {
    intakeMotor.set(0.6);
  }

  public void intakeIn() {
    intakeMotor.set(-0.8);
  }

  public void stopIndexer() {
    indexerMotor.set(0);
  }

  public void indexerIn() {
    indexerMotor.set(-0.18);
  }

  public void indexerInSlow() {
    indexerMotor.set(-0.01);
  }

  public void indexerInFast() {
    indexerMotor.set(-0.4);
  }

  public void indexerOut() {
    indexerMotor.set(0.3);
  }

  public void stopIntakeIndexer() {
    stopIntake();
    stopIndexer();
  }

  public void inIntakeIndexer() {
    intakeIn();
    indexerIn();
  }

  public void outIntakeIndexer() {
    intakeOut();
    indexerOut();
  }

  public LaserCan.Measurement getLaserCanMeasurement() {
    return laserCan.getMeasurement();
  }

  public boolean isNote() {
    if (getLaserCanMeasurement() == null || getLaserCanMeasurement().status != LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      return false;
    }

    return getLaserCanMeasurement().distance_mm / 1000.0 < Constants.AutoConstants.normalLaserCAN;
  }
}
