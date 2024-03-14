package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import frc.robot.Constants;

// import frc.robot.Constants;
/**
 * Controls the intake and the indexer in one file
 */
public class IntakeSubsystem extends SubsystemBase {
    private CANSparkMax intakeMotor;
    private CANSparkMax indexerMotor;
    public RelativeEncoder intakeEncoder;
    public RelativeEncoder indexerEncoder;
    /**
     * Creates an intake object
     */
    public IntakeSubsystem() {
        intakeMotor = new CANSparkMax(Constants.DriveConstants.kIntakeMotorCanId /*Change Device ID*/, MotorType.kBrushless);
        indexerMotor = new CANSparkMax( Constants.DriveConstants.kIntakeIndexCanId /*Change Device ID*/, MotorType.kBrushless);
        intakeEncoder = intakeMotor.getEncoder();
    }
    /**
     * Stops intake's spinning
     */
    public void stopIntake() {
        intakeMotor.set(0);
    }
    /**
     * Spins intake out
     */
    public void intakeOut() {
        intakeMotor.set(0.6);
    }
    /**
     * Spins intake in
     */
    public void intakeIn() {
        intakeMotor.set(-0.7);
    }
    /**
     * Stops the indexer
     */
    public void stopIndexer() {
        indexerMotor.set(0);
    }
    /**
     * Spins intake in
     */
    public void indexerIn() {
        indexerMotor.set(-0.4);
    }
    /**
     * Spins indexer Out
     */
    public void indexerOut() {
        indexerMotor.set(0.3);
    }
    /**
     * Stops intake and indexer
     */
    public void stopIntakeIndexer() {
        stopIntake();
        stopIndexer();
    }
    /**  
    * Spins both intake and indexer in
    */
    public void inIntakeIndexer() {
        intakeIn();
        indexerIn();
    }
    /**
     * Spins both intake and indexer out
     */
    public void outIntakeIndexer() {
        intakeOut();
        indexerOut();
    }
}