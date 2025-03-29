// package frc.robot.subsystems;

// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.config.SparkMaxConfig;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import java.util.function.DoubleSupplier;

// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.SparkRelativeEncoder;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.SparkBase.ResetMode;

// import frc.robot.Constants;

// public class Pivot extends SubsystemBase{
    
//     private static Pivot pivot = null;

//     private final SparkMax motorThree = new SparkMax(Constants.PIVOT_ID, MotorType.kBrushless);
//     private final RelativeEncoder encoder = motorThree.getEncoder();
    
//     public static Pivot getInstance() {
//         if (pivot == null) {
//             pivot = new Pivot();
//         }

//             return pivot;

//     }
//         private Pivot() {
//             SparkMaxConfig config = new SparkMaxConfig();
//             config.idleMode(IdleMode.kBrake);
//             config.inverted(true);
//             // config.encoder.positionConversionFactor(,)
//             motorThree.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
//             encoder.setPosition(0);
//         }

//     public void manual(double percent) {
//         if (encoder.getPosition() >= Units.degreesToRotations(-5) && percent > 0) {
//             stop();
//         }
//         motorThree.set(percent);
//     }

//     public Command manualCommand(DoubleSupplier percent) {
//         return run(() -> manual(percent.getAsDouble()));
//     }

//     public void stop() {
//         motorThree.set(0);
        
//     }

//     @Override
//     public void periodic() { 
//     }
// }