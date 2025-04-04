package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  private static Climber climber = null;

  private final SparkMax climbMotor = new SparkMax(Constants.CLIMB_MOTOR_ID, MotorType.kBrushless);

  public static Climber getInstance() {
    if (climber == null) {
      climber = new Climber();
    }

    return climber;
  }

  private Climber() {
    setDefaultCommand(stop());
  }

  public Command up() {
    return run(
        () -> {
          climbMotor.set(1.0);
        });
  }

  public Command down() {
    return run(
        () -> {
          climbMotor.set(-1.0);
        });
  }

  public Command stop() {
    return run(
        () -> {
          climbMotor.set(0);
        });
  }

  @Override
  public void periodic() {}
}
