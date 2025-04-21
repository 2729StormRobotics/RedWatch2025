package frc.robot.subsystems.drive;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class DriveConstants {

  // Driving Parameters - Note that these are not the maximum capable speeds of
  // the robot, rather the allowed maximum speeds
  public static final double periodicTime = 0.02;
  public static final double kMaxSpeedMetersPerSecond = 5;
  public static final double kAlignMaxAngularSpeed = 2.5 * Math.PI; // Not defined in source, retained from original
  public static final double kAlignMaxSpeed = 3.0; // Not defined in source, retained from original

  // public static final double kDirectionSlewRate = 1.2;
  // public static final double kMagnitudeSlewRate = 1.8;
  // public static final double kRotationalSlewRate = 2.0;
  // Values commented out in source, so we comment them here too

  // Chassis configuration
  public static final double kTrackWidthX = Units.inchesToMeters(26.5);
  public static final double kTrackWidthY = Units.inchesToMeters(26.5);
  public static final double kWheelBase = Units.inchesToMeters(26.5);
  public static final double kRobotWidth = Units.inchesToMeters(35); // Not in source, retained from original

  public static final SwerveDriveKinematics kDriveKinematics =
      new SwerveDriveKinematics(
          new Translation2d(kWheelBase / 2, kTrackWidthX / 2),
          new Translation2d(kWheelBase / 2, -kTrackWidthX / 2),
          new Translation2d(-kWheelBase / 2, kTrackWidthX / 2),
          new Translation2d(-kWheelBase / 2, -kTrackWidthX / 2));

  // Angular offsets of the modules relative to the chassis in radians
  public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
  public static final double kFrontRightChassisAngularOffset = 0;
  public static final double kBackLeftChassisAngularOffset = Math.PI;
  public static final double kBackRightChassisAngularOffset = Math.PI / 2;

  // SPARK MAX CAN IDs
  public static final int kFrontLeftDrivingCanId = 2;
  public static final int kRearLeftDrivingCanId = 6;
  public static final int kFrontRightDrivingCanId = 4;
  public static final int kRearRightDrivingCanId = 8;

  public static final int kFrontLeftTurningCanId = 1;
  public static final int kRearLeftTurningCanId = 5;
  public static final int kFrontRightTurningCanId = 3;
  public static final int kRearRightTurningCanId = 7;

  public static final boolean kGyroReversed = false; // Was commented out in source, uncommented here for completeness

  public static final double kMaxAccelerationMetersPerSecondSquared = 2.5;
  public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI * 1.5;
  public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI * 2;

  public static final double kPXController = 1; // Not in source, retained from original
  public static final double kPYController = 1; // Not in source, retained from original
  public static final double kPThetaController = 1; // Not in source, retained from original

  public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
      new TrapezoidProfile.Constraints(
          kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

  public static final PathConstraints kPathConstraints =
      new PathConstraints(
          kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared,
          kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

  public static final double kSlowModeConstant = 0.50;
  public static final double kPathplannerTurnAngleP = 3; // Not in source, retained from original
  public static final double kPathplannerTurnAngleI = 0; // Not in source, retained from original
  public static final double kPathplannerTurnAngleD = 0; // Not in source, retained from original

  public static final double kPathplannerTranslationP = 5; // Not in source, retained from original
  public static final double kPathplannerTranslationI = 0.0; // Not in source, retained from original
  public static final double kPathplannerTranslationD = 0.0; // Not in source, retained from original

  public static final double kTurnAngleTolerance = 0.05;
  public static final double kTurnAngleRateTolerance = 0.02;

  public static final double kAlignPositionTolerance = 0.03; // Not in source, retained from original
  public static final double kAlignRotationTolerance = Units.degreesToRadians(3); // Was `angleThreshold = 3` in source, renamed here

  public static final double kTurnAngleP = 0.9;
  public static final double kTurnAngleI = 0;
  public static final double kTurnAngleD = 0;

  public static final double kTranslationP = 3.50;
  public static final double kTranslationI = 0.0;
  public static final double kTranslationD = 0.0;
}