// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.GeometryUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
import frc.robot.subsystems.Vision;

import java.util.Optional;
import org.photonvision.PhotonCamera;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  public static DriveSubsystem m_robotDrive;
  // public final SendableChooser<Command> autoChooser;

  public static ShooterSubsystem m_shooter = new ShooterSubsystem();
  public static IntakeSubsystem m_Intake = new IntakeSubsystem();
  public static ArmSubsystem m_Arm = new ArmSubsystem();
  public static AutoAimSubsystem m_AutoAim = new AutoAimSubsystem();

  public static CommandXboxController m_driverController =
      new CommandXboxController(OIConstants.kDriverControllerPort);
  public static CommandXboxController m_operatorController =
      new CommandXboxController(OIConstants.kOperatorControllerPort);

  public static boolean slowMode = false;

  public long timeRumble = 0;

  public Trigger zeroLock;

  public static PhotonCamera camera = new PhotonCamera("photonvision");

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Needs to happen in this order for pathplanner

    m_robotDrive = new DriveSubsystem();
    // put all named commands in a register command
    NamedCommands.registerCommand("shootAuto()", AutoCommands.shootAuto());
    NamedCommands.registerCommand("IntakeForward()", AutoCommands.IntakeForward());
    NamedCommands.registerCommand("IntakeOnly()", AutoCommands.IntakeOnly());
    NamedCommands.registerCommand(
        "LaserCanIntake()", IntakeCommands.runIntakeUntilNoteSequentialCommand());
    NamedCommands.registerCommand("ShooterForward()", ShooterCommands.runShooterForwardTimed(1500));
    AutoCommands.loadAutos();

    // autoChooser = AutoBuilder.buildAutoChooser();

    // File folder = new File("src/main/deploy/pathplanner/autos");
    // for (File f : folder.listFiles()) {
    //   autoChooser.addOption(f.getName(), getPathPlannerAuto(f.getName().substring(0,
    // f.getName().length() - 5)));
    // }

    // SmartDashboard.putData("Auto Chooser", autoChooser);

    configureButtonBindings();

    DataLogManager.start();

    // Record both DS control and joystick data
    DriverStation.startDataLog(DataLogManager.getLog());

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () ->
                m_robotDrive.drive(
                    -MathUtil.applyDeadband(
                        m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(
                        m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(
                        m_driverController.getRightX(), OIConstants.kDriveDeadband),
                    true,
                    true,
                    false,
                    zeroLock.getAsBoolean()),
            m_robotDrive));
  }

  public Command makeRobotSlow() {
    return new InstantCommand(
        () -> {
          slowMode = !slowMode;
        });
  }

  public Command resetGyroTeleop() {
    return new InstantCommand(
        () -> {
          RobotContainer.m_robotDrive.m_gyro.reset();
        });
  }

  public Command rumbleGamePad(long time) {
    return new FunctionalCommand(
        () -> {
          timeRumble = System.currentTimeMillis();
          SmartDashboard.putNumber("TimeRumble", timeRumble);
        },
        () -> {
          m_driverController.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.5);
          SmartDashboard.putNumber("Time", System.currentTimeMillis());
        },
        interrupted -> {
          m_driverController.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0);
        },
        () -> System.currentTimeMillis() - time > timeRumble,
        m_robotDrive);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    Trigger ampPreset = new Trigger(m_operatorController.y());
    ampPreset.onTrue(ArmCommands.setArm(55));
    Trigger podiumPreset = new Trigger(m_operatorController.b());
    podiumPreset.onTrue(ArmCommands.setArm(16));
    Trigger restPreset = new Trigger(m_operatorController.a());
    restPreset.onTrue(ArmCommands.setArm(0));

    Trigger makeRobotSlow = new Trigger(m_driverController.a());
    makeRobotSlow.onTrue(makeRobotSlow());

    Trigger moveArmUp =
        new Trigger(() -> m_operatorController.getLeftY() > OIConstants.kArmDeadband);
    moveArmUp.whileTrue(ArmCommands.moveArmUp());
    Trigger moveArmDown =
        new Trigger(() -> m_operatorController.getLeftY() < -OIConstants.kArmDeadband);
    moveArmDown.whileTrue(ArmCommands.moveArmDown());
    Trigger stopArm =
        new Trigger(() -> Math.abs(m_operatorController.getLeftY()) < OIConstants.kArmDeadband);
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

    zeroLock = new Trigger(m_driverController.rightTrigger());
  }

  public Command getAutonomousCommand() {
    String autoName = "Four Note Auto";
    Pose2d startingPose = PathPlannerAuto.getStaringPoseFromAutoFile(autoName);
    Pose2d flipped;
    Optional<Alliance> alliance = DriverStation.getAlliance();

    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
      flipped = GeometryUtil.flipFieldPose(startingPose);
      m_robotDrive.resetOdometry(flipped);
    } else {
      m_robotDrive.resetOdometry(startingPose);
    }

    return new PathPlannerAuto(autoName);
    // return autoChooser.getSelected();
  }
}
/**
 * Use this to pass the autonomous command to the main {@link Robot} class.
 *
 * @return the command to run in autonomous
 */
