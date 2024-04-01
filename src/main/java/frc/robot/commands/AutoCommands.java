package frc.robot.commands;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutoCommands{

    public static PathPlannerAuto[] autos;
    public static String defaultAuto ="Two Note Auto";

    public static void loadAutos(){
        //Load all autos here in this file
        autos=new PathPlannerAuto[1];
        autos[0]= new PathPlannerAuto("Two Note Auto");
    }

    public static Command shootAuto(){
      return new ParallelCommandGroup(
        ShooterCommands.runShooterForwardTimed(1500),
        new SequentialCommandGroup(new WaitCommand(0.75), 
        IntakeCommands.runIntakeForwardTimed(750))
      );
    }

    public static Command IntakeOnly(){
        return IntakeCommands.runIntakeOnlyTimed(6000);
    }

    public static Command IntakeForward(){
        return new SequentialCommandGroup(
            IntakeCommands.runIntakeForwardTimed(1500),
            new WaitCommand(0.5)
        );
    }

    // public static Command pathPlannerStart(){
    //     PathPlannerPath path = PathPlannerPath.fromPathFile("start_center_straight");
    //      return new InstantCommand(() -> m_robotDrive.resetOdometry(getPathPose(path)))
    //     .andThen(new WaitCommand(0.1)
    //     );
    // }

}