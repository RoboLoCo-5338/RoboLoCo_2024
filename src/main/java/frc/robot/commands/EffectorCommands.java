package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.RobotContainer;

public class EffectorCommands {
//  public static long startingTime;
  
    public static Command effectorReverse() {
      return new InstantCommand(
        () -> RobotContainer.m_effector.effectorReverse(),
        RobotContainer.m_effector
      );
    }
    
    public static Command effectorForward() {
      return new InstantCommand(
        () -> RobotContainer.m_effector.effectorForward(),
        RobotContainer.m_effector
      );
    }

    public static Command effectorStop() {
      return new InstantCommand(
        () -> RobotContainer.m_effector.effectorStop(),
        RobotContainer.m_effector
      );
    }

    // public static Command autoEffector(int setpoint) {
    //   return new FunctionalCommand(
    //     () -> {},
    //     () -> RobotContainer.effector.setEffectorRef(setpoint),
    //     (interrupt) -> RobotContainer.effector.effectorStop(),
    //     () ->  Math.abs(setpoint-RobotContainer.effector.getEffectorPosition())<=1,
    //     RobotContainer.effector
    //   );
    // }

  //   public static Command timeEffectorForward(double time) {
  //     return new FunctionalCommand(
  //       () -> {startingTime = System.currentTimeMillis();},
  //       () -> RobotContainer.m_effector.effectorForward(),
  //       (interrupt) -> RobotContainer.m_effector.effectorStop(),
  //       () -> System.currentTimeMillis() - startingTime > time 
  //     );
  //   }

  //   public static Command timeEffectorReverse(double time) {
  //     return new FunctionalCommand(
  //       () -> {startingTime = System.currentTimeMillis();},
  //       () -> RobotContainer.m_effector.effectorReverse(),
  //       (interrupt) -> RobotContainer.m_effector.effectorStop(),
  //       () -> System.currentTimeMillis() - startingTime > time 
  //     );
  // }
}