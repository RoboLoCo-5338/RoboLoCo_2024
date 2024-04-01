// package frc.robot.commands;

//er WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;


public class IntakeCommands  {

  /** Creates a new ArmCommands. */
  // Move arm to preset height and stop when the height is reached.
 
  // public static Command setArmAbsolute(double setpoint) {
  //   return new PIDCommand(
  //     new PIDController(2, 0, 0), 
  //     () -> RobotContainer.effector.armAbsEncoder.getPosition(),
  //     RobotContainer.m_Arm.armHeights[(int)setpoint],
  //     output -> {RobotContainer.m_Arm.moveArm(output*2);},
  //     //() -> Math.abs(RobotContainer.m_Arm.armHeights[preset]-RobotContainer.m_Effector.getEffectorPosition()) <= 0.1,
  //     RobotContainer.m_Arm);
  // }

  private static long startTime;
  private static long intakestartTime;
  private static long indexerstartTime;
//command to set speed for arm
  public static Command moveIntakeIn(){
    return new InstantCommand(
      () -> RobotContainer.m_Intake.inIntakeIndexer(),
      RobotContainer.m_Intake
      );
  }

  public static Command moveIndexerInFast(){
    return new InstantCommand(
      () -> RobotContainer.m_Intake.indexerInFast(),
      RobotContainer.m_Intake);
  }

  // Command bindings for arm and elevator methods.
  public static Command moveIntakeOut(){
    return new InstantCommand(
      () -> RobotContainer.m_Intake.outIntakeIndexer(),
      RobotContainer.m_Intake
    );
  }

  public static Command stopIntake(){
    return new InstantCommand(
      () -> RobotContainer.m_Intake.stopIntakeIndexer(),
      RobotContainer.m_Intake
    );
  }

  
  public static Command runIntakeForwardTimed(long time){
    return new FunctionalCommand(() -> {
      RobotContainer.m_Intake.stopIntakeIndexer();
      startTime = System.currentTimeMillis();
    },() -> RobotContainer.m_Intake.inIntakeIndexer(),interrupted -> RobotContainer.m_Intake.stopIntakeIndexer(), () -> System.currentTimeMillis()-time>startTime, RobotContainer.m_Intake);
  }

  public static Command runIntakeOnlyTimed(long time){

   return new FunctionalCommand(() -> {
      RobotContainer.m_Intake.stopIntake();
      intakestartTime = System.currentTimeMillis();
    },() -> RobotContainer.m_Intake.intakeIn(),interrupted -> RobotContainer.m_Intake.stopIntake(), () -> System.currentTimeMillis()-time>intakestartTime, RobotContainer.m_Intake);
  }

  public static Command runIndexerOnlyTimed(long time){

   return new FunctionalCommand(() -> {
      RobotContainer.m_Intake.stopIndexer();
      indexerstartTime = System.currentTimeMillis();
    },() -> RobotContainer.m_Intake.indexerInFast(),interrupted -> RobotContainer.m_Intake.stopIndexer(), () -> System.currentTimeMillis()-time>indexerstartTime, RobotContainer.m_Intake);
  }

  public static Command runIntakeUntilNote(){
    return new FunctionalCommand(() -> {
      RobotContainer.m_Intake.stopIntakeIndexer(); }, 
      () -> RobotContainer.m_Intake.inIntakeIndexer(), interrupted -> RobotContainer.m_Intake.stopIntakeIndexer(), () -> RobotContainer.m_Intake.isNote(), RobotContainer.m_Intake);
  }


}