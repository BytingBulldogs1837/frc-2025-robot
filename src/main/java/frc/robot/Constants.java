package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int DRIVER_PORT = 1;
    public static final int OPERATOR_PORT = 0;
    public static final double DEADBAND = 0.05;
  }

  public static final double maxSpeed = Units.feetToMeters(10);

  public static final int ALGAE_ROLLERS = 3;
  public static final int PIVOT_ID = 11;
  public static final int CORAL_SOLENOID_CHANNEL = 0;

  public static final int MOTOR_THREE_ID = 10;
  public static final int CLIMB_MOTOR_ID = 9;
}
