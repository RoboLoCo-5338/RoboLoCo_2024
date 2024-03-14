package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase{
    private CANSparkMax shootMotor1;
    private CANSparkMax shootMotor2;
    private RelativeEncoder shootEncoder1;
    private RelativeEncoder shootEncoder2;
    private SparkPIDController shootController1;
    private SparkPIDController shootController2;


    /**
     * makes a subsystem of shooter
     */
    public ShooterSubsystem() {
        shootMotor1 = new CANSparkMax(Constants.DriveConstants.kShooter1CanId,MotorType.kBrushless);
        shootMotor2 = new CANSparkMax(Constants.DriveConstants.kShooter2CanId,MotorType.kBrushless);
        shootEncoder1 = shootMotor1.getEncoder();
        shootEncoder2 = shootMotor2.getEncoder();
        shootController1 = shootMotor1.getPIDController();
        shootController2 = shootMotor2.getPIDController();
    }

    /**
     * spins the shooter forward
     */
    public void shooterForward() {
        shootMotor1.set(0.85);
        shootMotor2.set(0.85);
    }

    /**
     * spins the shooter backwards
     */
    public void shooterReverse() {
        shootMotor1.set(-0.3);
        shootMotor2.set(-0.3);
    }

    /**
     * stops shooter
     */
    public void shooterStop() {
        shootMotor1.set(0);
        shootMotor2.set(0);
    }
    /**
     * @return gives the encoder position
     */
    public double getEncoderPosition(){
        return (Math.abs(shootEncoder1.getPosition())+Math.abs(shootEncoder2.getPosition())/2);
    }



}