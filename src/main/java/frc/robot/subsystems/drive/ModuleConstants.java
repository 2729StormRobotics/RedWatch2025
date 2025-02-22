package frc.robot.subsystems.drive;


import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.NeoMotorConstants;

public final class ModuleConstants {
  // The MAXSwerve module can be configured with one of three pinion gears: 12T,
  // 13T, or 14T.
  // This changes the drive speed of the module (a pinion gear with more teeth
  // will result in a
  // robot that drives faster).
  public static final int kDrivingMotorPinionTeeth = 14;

  // Invert the turning encoder, since the output shaft rotates in the opposite
  // direction of
  // the steering motor in the MAXSwerve Module.
  public static final boolean kTurningEncoderInverted = true;

  // Calculations required for driving motor conversion factors and feed forward
  public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
  public static final double kWheelDiameterMeters = Units.inchesToMeters(2.8669);
  public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;

  // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the
  // bevel pinion
  public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
  public static final double kDriveWheelFreeSpeedRps =
      (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction;

  public static final double kDrivingEncoderPositionFactor =
      (kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction; // meters
  public static final double kDrivingEncoderVelocityFactor =
      ((kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction) / 60.0; // meters per second

  public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
  public static final double kTurningEncoderVelocityFactor =
      (2 * Math.PI) / 60.0; // radians per second

  public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
  public static final double kTurningEncoderPositionPIDMaxInput =
      kTurningEncoderPositionFactor; // radians

  public static final double kDrivingP = .1;
  public static final double kDrivingI = 0;
  public static final double kDrivingD = 0;
  public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
  public static final double kDrivingMinOutput = -1;
  public static final double kDrivingMaxOutput = 1;

  // Turning PID will have to be changed for robot relative, use sysid one day
  public static final double kTurningP = 1.5;
  public static final double kTurningI = 0;
  public static final double kTurningD = 0.025;
  public static final double kTurningFF = 0;
  public static final double kTurningMinOutput = -1;
  public static final double kTurningMaxOutput = 1;

  public static final IdleMode kDrivingMotorIdleMode = IdleMode.kCoast;
  public static final IdleMode kTurningMotorIdleMode = IdleMode.kCoast;

  public static final int kDrivingMotorCurrentLimit = 35; // amps
  public static final int kTurningMotorCurrentLimit = 20; // amps
}
