// package frc.robot.subsystems;


// import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import frc.robot.Constants;

// public class Algae extends SubsystemBase {

//     private static Algae algae = null;

//     private final WPI_VictorSPX motorOne = new WPI_VictorSPX(Constants.ALGAE_ROLLERS);
    
//     public static Algae getInstance() {
//         if (algae == null) {
//             algae = new Algae();
//         }

//             return algae;

//     }
//         private Algae() {}

//         public void runForwards() {
//         motorOne.set(1.0);
       

//     }   


//     public void runBackwards() {
//             motorOne.set(-1.0);
            

//         }
    
//     public void stop() {
//         motorOne.set(0);
        

//     }

//     @Override
//     public void periodic() {
        
//     }
// }
   

