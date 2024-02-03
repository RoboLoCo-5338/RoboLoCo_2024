package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private CANSparkMax intakeMotor;
    public RelativeEncoder intakeEncoder;

    public static double hookChange = 0;
    public Intake(){
        intakeMotor = new CANSparkMax(Constants.DriveConstants.kArmCanId, MotorType.kBrushless);
        intakeMotor.setIdleMode(IdleMode.kBrake);
        intakeEncoder = intakeMotor.getEncoder();
        intakeMotor.setSmartCurrentLimit(40);
    }
        
    public void resetIntake() {
        intakeEncoder.setPosition(0);

    }

    public double getIntakePosition() {
        return intakeEncoder.getPosition();
    
    }

    public void stopIntake() {
        intakeMotor.set(0);
    }

    public void moveIntake(double speed) {
        intakeMotor.set(speed);
    }

 }