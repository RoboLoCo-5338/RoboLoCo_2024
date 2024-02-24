package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import frc.robot.Constants;

// import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private CANSparkMax intakeMotor;
    private CANSparkMax indexerMotor;
    public RelativeEncoder intakeEncoder;
    public RelativeEncoder indexerEncoder;

    public IntakeSubsystem() {
        intakeMotor = new CANSparkMax(Constants.DriveConstants.kIntakeMotorCanId /*Change Device ID*/, MotorType.kBrushless);
        indexerMotor = new CANSparkMax( Constants.DriveConstants.kIntakeIndexCanId /*Change Device ID*/, MotorType.kBrushless);
        intakeEncoder = intakeMotor.getEncoder();
    }

    public void stopIntake() {
        intakeMotor.set(0);
    }

    public void intakeOut() {
        intakeMotor.set(0.3);
    }

    public void intakeIn() {
        intakeMotor.set(-0.3);
    }

    public void stopIndexer() {
        indexerMotor.set(0);
    }

    public void indexerIn() {
        indexerMotor.set(0.3);
    }

    public void indexerOut() {
        indexerMotor.set(-0.3);
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
}