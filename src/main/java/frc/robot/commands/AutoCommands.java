package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.SwerveSubsystem;
import java.io.IOException;

/**
 * Container for autonomous command sequences used during the autonomous period.
 */
public class AutoCommands {
  private AutoCommands() {
    // Utility class - prevent instantiation since all methods are static
  }

  /**
   * Mid Score autonomous routine that follows the Mid path and then drops the coral.
   *
   * @param swerve The swerve subsystem
   * @param coral The coral subsystem
   * @return A command sequence that follows the Mid path and drops coral
   */
  public static Command midScore(SwerveSubsystem swerve, Coral coral) {
    try {
      // Load the Mid path from PathPlanner
      PathPlannerPath path = PathPlannerPath.fromPathFile("Mid");
      
      // Get the initial pose from the path
      Pose2d initialPose = FlippingUtil.flipFieldPose(path.getStartingHolonomicPose().get());
      
      // Create a sequence to set odometry, follow the path, then drop coral
      return Commands.sequence(
          // First set the odometry to match the path's starting position
          Commands.runOnce(() -> swerve.resetOdometry(initialPose)),
          // Then follow the path
          AutoBuilder.followPath(path),
          Commands.waitSeconds(0.5), // Short pause
          coral.down().withTimeout(10.0)
      ).withName("Mid Score Auto");
    } catch (IOException e) {
      // Handle file not found or IO error
      return Commands.print("ERROR: Could not load Mid path file")
          .withName("Mid Score Error");
    } catch (Exception e) {
      // Handle other exceptions
      return Commands.print("ERROR: " + e.getMessage())
          .withName("Mid Score Error");
    }
  }

  /**
   * Simple command that just sets odometry to the starting point of the Mid path
   * without actually following the path.
   *
   * @param swerve The swerve subsystem
   * @return A command that sets odometry to the path's initial pose
   */
  public static Command setOdometryOnly(SwerveSubsystem swerve) {
    try {
      // Load the same Mid path to get the initial position
      PathPlannerPath path = PathPlannerPath.fromPathFile("Mid");
      
      // Get the initial pose from the path
      Pose2d initialPose = FlippingUtil.flipFieldPose(path.getStartingHolonomicPose().get());
      
      // Just set the odometry without following the path
      return Commands.runOnce(() -> swerve.resetOdometry(initialPose))
          .withName("Set Odometry Only");
    } catch (IOException e) {
      // Handle file not found or IO error
      return Commands.sequence(
          Commands.print("ERROR: Could not load Mid path file"),
          // Fall back to origin if path can't be loaded
          Commands.runOnce(() -> swerve.resetOdometry(new Pose2d()))
      ).withName("Set Odometry Error");
    } catch (Exception e) {
      // Handle other exceptions
      return Commands.sequence(
          Commands.print("ERROR: " + e.getMessage()),
          // Fall back to origin if there's another error
          Commands.runOnce(() -> swerve.resetOdometry(new Pose2d()))
      ).withName("Set Odometry Error");
    }
  }

  /**
   * Creates a simple "do nothing" command for when no autonomous action is desired.
   * 
   * @return A command that does nothing
   */
  public static Command doNothing() {
    return Commands.none().withName("Do Nothing");
  }
}
