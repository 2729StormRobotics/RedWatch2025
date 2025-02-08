// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.PhotonVision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;


public class VisionConstants extends SubsystemBase {
  /** Creates a new VisionConstants. */
  public VisionConstants() {}

  public static final String cam1Name = "Front_Camera";
  public static final String cam3Name = "Left_Camera";
  public static final String cam2Name = "Right_Camera";

  // Change values to the actual distance from the center of the robot, change rotation if needed.
  public static final Transform3d cam1RobotToCam = new Transform3d(
                        new Translation3d(
                                        Units.inchesToMeters(-9),
                                        Units.inchesToMeters(7),
                                        Units.inchesToMeters(10)),
                        new Rotation3d(0, 
                        Rotation2d.fromDegrees(0).getRadians(), 
                        Rotation2d.fromDegrees(0).getRadians()
                        ));
  public static final Transform3d cam3RobotToCam = new Transform3d(
                        new Translation3d(
                                Units.inchesToMeters(-0.25),
                                Units.inchesToMeters(4.5),
                                Units.inchesToMeters(11)
                        ),
                        new Rotation3d(0, 
                                Rotation2d.fromDegrees(90 - 61.90).getRadians(), 
                                Rotation2d.fromDegrees(90).getRadians()));// maybe need to change

  public static final Transform3d cam2RobotToCam = new Transform3d(
                        new Translation3d(
                                Units.inchesToMeters(-0.25),
                                Units.inchesToMeters(-4.5),
                                Units.inchesToMeters(11)
                        ),
                        new Rotation3d(0, 
                                Rotation2d.fromDegrees(90 - 61.90).getRadians(), 
                                Rotation2d.fromDegrees(-90).getRadians())); // maybe need to change


  // The layout of the AprilTags on the field
  public static final AprilTagFieldLayout kTagLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();

  // Change values as needed **CHANGE**
  public static final double AMBIGUITY_THRESHOLD = 0.5;
  public static final double MAX_DISTANCE = 4; // meters

  // The standard deviations of our vision estimated poses, which affect
  // correction rate
  // (Fake values. Experiment and determine estimation noise on an actual robot.) **CHANGE**
  public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(2, 2, 8);
  public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.2, 0.2, 1);

  // Provides a simulated version of a given Transform3d by removing rotations about the X and Y axes.
  public static Transform3d getSimVersion(Transform3d real) {
    return new Transform3d(
      real.getTranslation(),
      new Rotation3d(
              0,
              0,
              real.getRotation().getZ()
      )
    );
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}