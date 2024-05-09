// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ArmCommands;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.AutoAimSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.List;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.AutoBuilderException;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.GeometryUtil;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  public static  DriveSubsystem m_robotDrive;
  public final SendableChooser<Command> autoChooser = null;

  public static ShooterSubsystem m_shooter = new ShooterSubsystem();
  public static IntakeSubsystem m_Intake = new IntakeSubsystem();
  public static ArmSubsystem m_Arm = new ArmSubsystem();
  public static AutoAimSubsystem m_AutoAim = new AutoAimSubsystem();
  
public static CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
public static CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);

   public static boolean slowMode = false;

public long timeRumble=0;



  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    
    //Needs to happen in this order for pathplanner

    m_robotDrive = new DriveSubsystem();
    //put all named commands in a register command
    NamedCommands.registerCommand("shootAuto()",AutoCommands.shootAuto());
    NamedCommands.registerCommand("IntakeForward()",AutoCommands.IntakeForward());
    NamedCommands.registerCommand("IntakeOnly()",AutoCommands.IntakeOnly());
    NamedCommands.registerCommand("LaserCanIntake()", IntakeCommands.runIntakeUntilNoteSequentialCommand());
    NamedCommands.registerCommand("ShooterForward()", ShooterCommands.runShooterForwardTimed(1500));
    AutoCommands.loadAutos();


  //   public static Command pathPlannerStart(){
  //     PathPlannerPath path = PathPlannerPath.fromPathFile("start_center_straight");
  //      return new InstantCommand(() -> m_robotDrive.resetOdometry(getPathPose(path)))
  //     .andThen(new WaitCommand(0.1)
  //     );
  // }


  // autoChooser = AutoBuilder.buildAutoChooser();
   ////autoChooser.addOption("Two Note", AutoCommands.autos[0]);
   // SmartDashboard.putData("Auto Chooser", autoChooser);
    // Configure the button bindings
    configureButtonBindings();


     DataLogManager.start();

  // Record both DS control and joystick data
  DriverStation.startDataLog(DataLogManager.getLog());


    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true, true,false),
            m_robotDrive));
  }

   public Command makeRobotSlow(){
    return new InstantCommand(() -> { slowMode= !slowMode;});
  }

  public Command resetGyroTeleop(){
    return new InstantCommand(() -> {RobotContainer.m_robotDrive.m_gyro.reset();});
  }

  public Command rumbleGamePad(long time){
    return new FunctionalCommand(()->{timeRumble=System.currentTimeMillis();  SmartDashboard.putNumber("TimeRumble", timeRumble);},
    ()->{ m_driverController.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.5);
    SmartDashboard.putNumber("Time", System.currentTimeMillis());},
      interrupted ->{ m_driverController.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0);}, 
      ()-> System.currentTimeMillis()-time>timeRumble,m_robotDrive);
  }

  // public static Command rumbleGamePad(long time){
  //   return new FunctionalCommand(() -> {
  //     RobotContainer.m_Intake.stopIntakeIndexer();
  //     startTime = System.currentTimeMillis();
  //   },() -> RobotContainer.m_Intake.inIntakeIndexer(),
  //   interrupted -> RobotContainer.m_Intake.stopIntakeIndexer(),
  //    () -> System.currentTimeMillis()-time>startTime,
  //     RobotContainer.m_Intake);
  // }


  

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    Trigger ampPreset = new Trigger(m_operatorController.y());
    ampPreset.onTrue(ArmCommands.setArm(55));
    Trigger podiumPreset = new Trigger(m_operatorController.b());
    podiumPreset.onTrue(ArmCommands.setArm(16));
    Trigger restPreset = new Trigger(m_operatorController.a());
    restPreset.onTrue(ArmCommands.setArm(0));

     Trigger makeRobotSlow = new Trigger(m_driverController.rightTrigger());
     makeRobotSlow.onTrue(makeRobotSlow());

    Trigger moveArmUp = new Trigger(() -> m_operatorController.getLeftY()> OIConstants.kArmDeadband);
    moveArmUp.whileTrue(ArmCommands.moveArmUp());
    Trigger moveArmDown = new Trigger(() -> m_operatorController.getLeftY()< -OIConstants.kArmDeadband);
    moveArmDown.whileTrue(ArmCommands.moveArmDown());
    Trigger stopArm = new Trigger(() -> Math.abs(m_operatorController.getLeftY())<OIConstants.kArmDeadband);
    stopArm.whileTrue(ArmCommands.stopArm());

    Trigger intakeIn = new Trigger(m_operatorController.rightTrigger());
    intakeIn.whileTrue(IntakeCommands.moveIntakeIn());
    intakeIn.onFalse(IntakeCommands.stopIntake());

    Trigger intakeOut = new Trigger(m_operatorController.leftTrigger());
    intakeOut.whileTrue(IntakeCommands.moveIntakeOut());
    intakeOut.onFalse(IntakeCommands.stopIntake());

    Trigger shootOut = new Trigger(m_operatorController.rightBumper());
    shootOut.whileTrue(ShooterCommands.shooterForward());
    shootOut.onFalse(ShooterCommands.shooterStop());

    Trigger shootIn = new Trigger(m_operatorController.leftBumper());
    shootIn.whileTrue(ShooterCommands.shooterReverse());
    shootIn.onFalse(ShooterCommands.shooterStop());

    // reset gyro button binding 
    Trigger resetGyro = new Trigger(m_driverController.x());
    resetGyro.onTrue(resetGyroTeleop());


   

  }

  public Command getAutonomousCommand() {
    Pose2d startingPose = PathPlannerAuto.getStaringPoseFromAutoFile("Two Note");
    Pose2d flipped;
    Optional<Alliance> alliance = DriverStation.getAlliance();
      if(alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
        flipped=GeometryUtil.flipFieldPose(startingPose);
         m_robotDrive.resetOdometry(flipped);
      }else{
         m_robotDrive.resetOdometry(startingPose);
      }
   
    return new PathPlannerAuto("Two Note");
    
   // autoChoose/,mr.getSelected();5
  }
}
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
// public Command getAutonomousCommands(){
//   //
// }
