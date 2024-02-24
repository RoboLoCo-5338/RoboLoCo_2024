package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase{
    private CANSparkMax shootMotor1;
    private CANSparkMax shootMotor2;

    public ShooterSubsystem() {
        shootMotor1 = new CANSparkMax(Constants.DriveConstants.kShooter1CanId,MotorType.kBrushless);
        shootMotor2 = new CANSparkMax(Constants.DriveConstants.kShooter2CanId,MotorType.kBrushless);
    }

    public void shooterForward() {
        shootMotor1.set(0.3);
        shootMotor2.set(-0.3);
    }

    public void shooterReverse() {
        shootMotor1.set(-0.3);
        shootMotor2.set(0.3);
    }

    public void shooterStop() {
        shootMotor1.set(0);
        shootMotor2.set(0);
    }

}