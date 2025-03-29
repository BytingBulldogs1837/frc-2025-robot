// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
 
package frc.robot;

//import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Radian;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.*/
 
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 1;
    public static final double DEADBAND = 0.05;
  } 

  public static final double maxSpeed = Units.feetToMeters(10);

  public static final int ALGAE_ROLLERS = 3;
  public static final int PIVOT_ID = 11;

  public static final int MOTOR_THREE_ID = 10;
  public static final int MOTOR_FOUR_ID = 9;

   //public static final class ApriltagConstants {
  //   public static final class ApriltagCameraConfig {
  //     private String name;
  //     private Transform3d transform;
  //     private PoseStrategy strategy;

  //     public ApriltagCameraConfig(
  //         String name,
  //         Transform3d transform,
  //         PoseStrategy strategy) {
  //       this.name = name;
  //       this.transform = transform;
  //       this.strategy = strategy;
  //     }

  //     public String getName() {
  //       return name;
  //     }

  //     public Transform3d getTransform() {
  //       return transform;
  //     }

  //     public PoseStrategy getStrategy() {
  //       return strategy;
  //     }
    

//     public static final ApriltagCameraConfig[] PHOTON_CAMERAS = {
//         // new ApriltagCameraConfig(
//         // "Front Right",
//         // new Transform3d(
//         // new Translation3d(
//         // 0.2794,
//         // 0.2794,
//         // 0.264),
//         // new Rotation3d(
//         // 0,
//         // 0,
//         // Degree.of(-45).in(Radian))),
//         // PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR),
//         new ApriltagCameraConfig(
//             "Back",
//             new Transform3d(
//                 new Translation3d(
//                     0.16,
//                     0.27,
//                     0.585),
//                 new Rotation3d(
//                     0,
//                     0,
//                     Degree.of(180).in(Radian))),
//             PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR),
//     };

//     public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFields.k2025ReefscapeWelded
//         .loadAprilTagLayoutField();

//     /** Maximum allowed ambiguity for pose estimation (0-1, lower is better) */
//     public static final double MAXIMUM_AMBIGUITY = 0.25;

//     /** Standard deviations for single tag pose estimation */
//     public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(6, 6, 12);

//     /** Standard deviations for multi-tag pose estimation */
//     public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(1, 1, 2);

//     /** Debounce time for camera reads in seconds */
//     public static final double CAMERA_DEBOUNCE_TIME = 0.150;

//     /** Maximum number of camera results to keep in memory */
//     public static final int MAX_CAMERA_RESULTS = 5;
//   }
 }
