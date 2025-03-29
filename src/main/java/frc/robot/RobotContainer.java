// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants;
// import frc.robot.commands.AlgaeIn;
// import frc.robot.commands.AlgaeOut;
import frc.robot.commands.ClimberIn;
import frc.robot.commands.ClimberOut;
import frc.robot.commands.CoralIn;
import frc.robot.commands.CoralOut;
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666import frc.robot.subsystems.Apriltag;
// import frc.robot.subsystems.Algae;
//import frc.robot.commands.AlgaeIn;
//import frc.robot.commands.AlgaeOut;
// import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Coral;
// import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
//import edu.wpi.first.math.geometry.Translation2d;
//import edu.wpi.first.math.kinematics.ChassisSpeeds;
//import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
//import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final SwerveSubsystem drivebase = new SwerveSubsystem();

  private final Coral coral = Coral.getInstance();
  // private final Apriltag apriltag = Apriltag.getInstance(drivebase);
  private CoralIn coralIn;
  private CoralOut coralOut;
  private boolean isRed = false;
  // Replace with CommandPS4Controller or CommandJoystick if needed
  // private final XboxController m_driverController = new XboxController(1);

  private final CommandXboxController m_driverController = new CommandXboxController(
      Constants.OperatorConstants.kDriverControllerPort);
  private final CommandXboxController operator = new CommandXboxController(0);

  // Drive control config
  private final SwerveInputStream swerveInputStream = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> isRed ? m_driverController.getLeftY() : -m_driverController.getLeftY(),
      () -> isRed ? m_driverController.getLeftX() : -m_driverController.getLeftX())
      .withControllerRotationAxis(() -> -m_driverController.getRightX())
      .deadband(Constants.OperatorConstants.DEADBAND)
      .scaleTranslation(1.0)
      .allianceRelativeControl(true);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    coralIn = new CoralIn(coral);
    coralOut = new CoralOut(coral);
    CameraServer.startAutomaticCapture();
    NamedCommands.registerCommand("score", new CoralOut(coral).withTimeout(1));
    configureBindings();
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      isRed = alliance.get() == DriverStation.Alliance.Red;
    }
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    Command driveInputStream = drivebase.driveFieldOriented(swerveInputStream);
    drivebase.setDefaultCommand(driveInputStream);
    // Pivot.getInstance().setDefaultCommand(Pivot.getInstance().manualCommand(() ->
    // -operator.getLeftY() / 3.0));
    operator.rightBumper().whileTrue(new CoralIn(Coral.getInstance()));
    operator.leftBumper().whileTrue(new CoralOut(Coral.getInstance()));
    operator.start().whileTrue(new ClimberIn(Climber.getInstance()));
    operator.b().whileTrue(new ClimberOut(Climber.getInstance()));
    operator.y().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(Pose2d.kZero)));
    
    // operator.rightTrigger().whileTrue(new AlgaeIn(Algae.getInstance()));
    // operator.leftTrigger().whileTrue(new AlgaeOut(Algae.getInstance()));
    // operator.y().whileTrue(new AlgaeIn(Algae.getInstance()));
    // operator.x().whileTrue(new AlgaeOut(Algae.getInstance()));
  }

  /*
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return drivebase.driveCommand(4.6,2);

    return Commands.sequence(
        drivebase.driveRobotOriented(() -> new Translation2d(0.7 * (isRed ? -1 : 1), 0), () -> 0.0, () -> true).withTimeout(0.5),
        drivebase.driveRobotOriented(() -> new Translation2d(0.0, 0), () -> 0.0, () -> true));
    // return drivebase.getAutonomousCommand("New Auto");

  }
}