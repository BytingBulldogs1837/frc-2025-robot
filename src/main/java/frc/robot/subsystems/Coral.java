package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Coral extends SubsystemBase {

    private static Coral coral = null;

    // private final SparkMax motorThree = new SparkMax(Constants.MOTOR_THREE_ID, MotorType.kBrushless);
    private final Solenoid gate = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
    private boolean gateup = false;
    


    
    public static Coral getInstance() {
        if (coral == null) {
            coral = new Coral();
        }

            return coral;

    }
        private Coral() {}

        public void Up() {
        gateup = true;
       

    }   


    public void Down() {
    gateup = false;

        
    }

    @Override
    public void periodic() {
    gate.set(gateup);

    }
}
    

