package frc.robot.commands;

import com.pathplanner.lib.commands.PathPlannerAuto;

public class AutoCommands{

    public static PathPlannerAuto[] autos;
    public static String defaultAuto ="null";

    public static void loadAutos(){
        //Load all autos here in this file
        autos=new PathPlannerAuto[1];
        autos[0]= new PathPlannerAuto("null");
    }
}