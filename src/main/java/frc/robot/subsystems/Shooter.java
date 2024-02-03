package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    
    private CANSparkMax shootMotor;

    public Shooter() {
        shootMotor = new CANSparkMax(Constants.DriveConstants.kFlywheelCanId, MotorType.kBrushless);
        // shootMotor.setIdleMode(IdleMode.kBrake);
        // shootMotor.setSmartCurrentLimit(40);

    }

    public void shooterForward() {//shoot
        //SmartDashboard.putString("PLEASE", "WORK");
        shootMotor.set(0.9 );
        
    }

    public void shooterReverse() {//retract ball
        shootMotor.set(-0.3);

    }

    public void shooterStop() {
        shootMotor.set(0);
    }



}