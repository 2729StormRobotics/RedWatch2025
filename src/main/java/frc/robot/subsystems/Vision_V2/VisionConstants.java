// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision_V2;

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
        public VisionConstants() {
        }

        public static final String[] camNames = {
                        "Intake", "Outtake", "Intake Up"
        };
        public static final int numCameras = camNames.length;

        public static final Transform3d[] camsRobotToCam = {
                        new Transform3d(
                                        new Translation3d(
                                                        Units.inchesToMeters(13.000000),
                                                        Units.inchesToMeters(4.5),
                                                        Units.inchesToMeters(5.6000)),
                                        new Rotation3d(0,
                                                        Rotation2d.fromDegrees(35).getRadians(),
                                                        Rotation2d.fromDegrees(0).getRadians())),
                        new Transform3d(
                                        new Translation3d(
                                                        Units.inchesToMeters(-8),
                                                        Units.inchesToMeters(4.5),
                                                        Units.inchesToMeters(11.85)),
                                        new Rotation3d(0,
                                                        Rotation2d.fromDegrees(0).getRadians(),
                                                        Rotation2d.fromDegrees(0).getRadians())),
                        new Transform3d(
                                        new Translation3d(
                                                        Units.inchesToMeters(8),
                                                        Units.inchesToMeters(3.5),
                                                        Units.inchesToMeters(24.850)),

                                        new Rotation3d(0,
                                                        Rotation2d.fromDegrees(0).getRadians(),
                                                        Rotation2d.fromDegrees(0).getRadians())),
        };

        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout
                        .loadField(AprilTagFields.k2025ReefscapeWelded);

        public static final double AMBIGUITY_THRESHOLD = 0.2;
        public static final double MAX_DISTANCE = 4; // meters

        // The standard deviations of our vision estimated poses, which affect
        // correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(1, 1, 20);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.2, 0.2, 20);

        public static Transform3d getSimVersion(Transform3d real) {
                return new Transform3d(real.getTranslation(), new Rotation3d(0, 0, real.getRotation().getZ()));
        }
}