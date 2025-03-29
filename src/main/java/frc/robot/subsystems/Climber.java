package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

 

    public class Climber extends SubsystemBase {

    private static Climber climber = null;

    private final SparkMax motorFour = new SparkMax(Constants.MOTOR_FOUR_ID, MotorType.kBrushless);
    


    
    public static Climber getInstance() {
        if (climber == null) {
            climber = new Climber();
        }

            return climber;

    }
        private Climber() {}

        public void runForwards() {
        motorFour.set(0.5);
       

    }   


    public void runBackwards() {
            motorFour.set(-0.5);
            

        }
    
    public void stop() {
        motorFour.set(0);
        
    }

    @Override
    public void periodic() {
        
    }
}
    



