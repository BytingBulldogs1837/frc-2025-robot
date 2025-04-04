// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoCommands;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final SwerveSubsystem drivebase = new SwerveSubsystem();

  private final Coral coral = Coral.getInstance();
  private final Climber climber = Climber.getInstance();

  private final CommandXboxController driver =
      new CommandXboxController(OperatorConstants.DRIVER_PORT);
  private final CommandXboxController operator =
      new CommandXboxController(OperatorConstants.OPERATOR_PORT);

  // Drive control config
  private final SwerveInputStream swerveInputStream =
      SwerveInputStream.of(
              drivebase.getSwerveDrive(), () -> -driver.getLeftY(), () -> -driver.getLeftX())
          .withControllerRotationAxis(() -> -driver.getRightX())
          .deadband(OperatorConstants.DEADBAND)
          .scaleTranslation(1.0)
          .allianceRelativeControl(true);

  // Auto command selector
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices */
  public RobotContainer() {
    CameraServer.startAutomaticCapture();
    configureBindings();
    configureAutoChooser();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Driver bindings
    Command driveInputStream = drivebase.driveFieldOriented(swerveInputStream);
    drivebase.setDefaultCommand(driveInputStream);

    // Operator bindings
    operator.a().whileTrue(coral.down());
    operator.leftBumper().whileTrue(climber.up());
    operator.rightBumper().whileTrue(climber.down());
    operator.y().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(Pose2d.kZero)));
  }

  /**
   * Configure the SendableChooser for autonomous command selection.
   */
  private void configureAutoChooser() {
    // Add available auto commands to the chooser
    autoChooser.setDefaultOption("Do Nothing", AutoCommands.doNothing());
    autoChooser.addOption("Mid Score", AutoCommands.midScore(drivebase, coral));
    autoChooser.addOption("Set Odometry Only", AutoCommands.setOdometryOnly(drivebase));
    
    // Put the chooser on the dashboard
    SmartDashboard.putData("Auto Mode", autoChooser);
  }

  /**
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
