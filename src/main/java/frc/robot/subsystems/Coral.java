package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Coral extends SubsystemBase {
  private static Coral coral = null;

  private final Solenoid gate =
      new Solenoid(PneumaticsModuleType.CTREPCM, Constants.CORAL_SOLENOID_CHANNEL);

  public static Coral getInstance() {
    if (coral == null) {
      coral = new Coral();
    }

    return coral;
  }

  private Coral() {
    setDefaultCommand(up());
  }

  public Command up() {
    return run(
        () -> {
          gate.set(false);
        });
  }

  public Command down() {
    return run(
        () -> {
          gate.set(true);
        });
  }
}
