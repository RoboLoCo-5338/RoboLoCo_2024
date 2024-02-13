package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;


// import frc.robot.Constants;

public class IntakeIndexer {
    private CANSparkMax intakeMotor;
    private CANSparkMax indexerMotor;
    public RelativeEncoder intakeEncoder;
    public RelativeEncoder indexerEncoder;

    public IntakeIndexer() {
        intakeMotor = new CANSparkMax(0 /*Change Device ID*/, MotorType.kBrushless);
        indexerMotor = new CANSparkMax( 1 /*Change Device ID*/, MotorType.kBrushless);
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