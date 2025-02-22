// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.kFrontLeftDrivingCanId;
import static frc.robot.subsystems.drive.DriveConstants.kFrontLeftTurningCanId;
import static frc.robot.subsystems.drive.DriveConstants.kFrontRightDrivingCanId;
import static frc.robot.subsystems.drive.DriveConstants.kFrontRightTurningCanId;
import static frc.robot.subsystems.drive.DriveConstants.kRearLeftDrivingCanId;
import static frc.robot.subsystems.drive.DriveConstants.kRearLeftTurningCanId;
import static frc.robot.subsystems.drive.DriveConstants.kRearRightDrivingCanId;
import static frc.robot.subsystems.drive.DriveConstants.kRearRightTurningCanId;
import static frc.robot.subsystems.drive.ModuleConstants.kDrivingEncoderPositionFactor;
import static frc.robot.subsystems.drive.ModuleConstants.kDrivingEncoderVelocityFactor;
import static frc.robot.subsystems.drive.ModuleConstants.kDrivingMotorCurrentLimit;
import static frc.robot.subsystems.drive.ModuleConstants.kDrivingMotorIdleMode;
import static frc.robot.subsystems.drive.ModuleConstants.kTurningEncoderPositionFactor;
import static frc.robot.subsystems.drive.ModuleConstants.kTurningEncoderVelocityFactor;
import static frc.robot.subsystems.drive.ModuleConstants.kTurningMotorCurrentLimit;
import static frc.robot.subsystems.drive.ModuleConstants.kTurningMotorIdleMode;
import static frc.robot.subsystems.drive.ModuleConstants.kWheelDiameterMeters;

import com.revrobotics.spark.SparkLowLevel.PeriodicFrame;
import com.pathplanner.lib.config.ModuleConfig;
import com.revrobotics.AbsoluteEncoder;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

import java.util.Queue;

/**
 * Module IO implementation for SparkMax drive motor controller, SparkMax turn
 * motor controller (NEO
 * or NEO 550), and analog absolute encoder connected to the RIO
 *
 * <p>
 * NOTE: This implementation should be used as a starting point and adapted to
 * different hardware
 * configurations (e.g. If using a CANcoder, copy from "ModuleIOTalonFX")
 *
 * <p>
 * To calibrate the absolute encoder offsets, point the modules straight (such
 * that forward
 * motion on the drive motor will propel the robot forward) and copy the
 * reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOSparkMax implements ModuleIO {

  private final SparkMax driveSparkMax;
  private final SparkMax turnSparkMax;

  private SparkMaxConfig driveConfig;
  private SparkMaxConfig turnConfig;
  private final SparkClosedLoopController drivePIDController;
  private final SparkClosedLoopController turnPIDController;

  private final RelativeEncoder driveEncoder;
  private final AbsoluteEncoder turnAbsoluteEncoder;

  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  private final boolean isTurnMotorInverted = true;
  private final double absoluteEncoderOffset;

  public ModuleIOSparkMax(int index) {
    switch (index) {
      case 0: // Front Left
        driveSparkMax = new SparkMax(kFrontLeftDrivingCanId, MotorType.kBrushless);
        turnSparkMax = new SparkMax(kFrontLeftTurningCanId, MotorType.kBrushless);
        absoluteEncoderOffset = DriveConstants.kFrontLeftChassisAngularOffset; // MUST BE CALIBRATED
        break;
      case 1: // Front Right
        driveSparkMax = new SparkMax(kFrontRightDrivingCanId, MotorType.kBrushless);
        turnSparkMax = new SparkMax(kFrontRightTurningCanId, MotorType.kBrushless);
        absoluteEncoderOffset = DriveConstants.kFrontRightChassisAngularOffset; // MUST BE CALIBRATED
        break;
      case 2: // Back Left
        driveSparkMax = new SparkMax(kRearLeftDrivingCanId, MotorType.kBrushless);
        turnSparkMax = new SparkMax(kRearLeftTurningCanId, MotorType.kBrushless);
        absoluteEncoderOffset = DriveConstants.kBackLeftChassisAngularOffset; // MUST BE CALIBRATED
        break;
      case 3: // Back Right
        driveSparkMax = new SparkMax(kRearRightDrivingCanId, MotorType.kBrushless);
        turnSparkMax = new SparkMax(kRearRightTurningCanId, MotorType.kBrushless);
        absoluteEncoderOffset = DriveConstants.kBackRightChassisAngularOffset; // MUST BE CALIBRATED
        break;
      default:
        throw new RuntimeException("Invalid module index");
    }
    driveSparkMax.configure(MotorConfigs.drivingConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    turnSparkMax.configure(MotorConfigs.turningConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    driveSparkMax.setCANTimeout(0);
    turnSparkMax.setCANTimeout(0);

    driveEncoder = driveSparkMax.getEncoder();
    turnAbsoluteEncoder = turnSparkMax.getAbsoluteEncoder();
    
    drivePIDController = driveSparkMax.getClosedLoopController();
    turnPIDController = turnSparkMax.getClosedLoopController();

    // Log things?
    timestampQueue = SparkMaxOdometryThread.getInstance().makeTimestampQueue();
    drivePositionQueue = SparkMaxOdometryThread.getInstance().registerSignal(driveEncoder::getPosition);
    turnPositionQueue = SparkMaxOdometryThread.getInstance().registerSignal(turnAbsoluteEncoder::getPosition);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.drivePositionRad = driveEncoder.getPosition() / (kWheelDiameterMeters / 2);
    inputs.drivePositionMeters = driveEncoder.getPosition();
    inputs.driveVelocityMeterPerSec = driveEncoder.getVelocity();
    inputs.driveVelocityRadPerSec = driveEncoder.getVelocity() / (kWheelDiameterMeters / 2);
    inputs.driveAppliedVolts = driveSparkMax.getAppliedOutput() * driveSparkMax.getBusVoltage();
    inputs.driveCurrentAmps = new double[] { driveSparkMax.getOutputCurrent() };

    inputs.turnAbsolutePosition = getTurnPosition();
    inputs.turnPosition = getTurnPosition();
    inputs.turnVelocityRadPerSec = turnAbsoluteEncoder.getVelocity();
    inputs.turnAppliedVolts = turnSparkMax.getAppliedOutput() * turnSparkMax.getBusVoltage();
    inputs.turnCurrentAmps = new double[] { turnSparkMax.getOutputCurrent() };

    inputs.odometryTimestamps = timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad = drivePositionQueue.stream()
        .mapToDouble((Double value) -> Units.rotationsToRadians(value))
        .toArray();
    inputs.odometryTurnPositions = turnPositionQueue.stream()
        .map((Double value) -> Rotation2d.fromRotations(value))
        .toArray(Rotation2d[]::new);
    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    drivePIDController.setReference(velocityRadPerSec, SparkMax.ControlType.kVelocity);
  }

  @Override
  public void setTurnPosition(double angle) {
    turnPIDController.setReference(
        angle + absoluteEncoderOffset, SparkMax.ControlType.kPosition);
  }

  public Rotation2d getTurnPosition() {
    double angle = turnAbsoluteEncoder.getPosition() - absoluteEncoderOffset;

    return Rotation2d.fromRadians(angle);
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveSparkMax.setVoltage(volts);
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnSparkMax.setVoltage(volts);
  }
  @Override
  public double getTurnPositionError(double angle) {
    return Math.abs(turnAbsoluteEncoder.getPosition() - angle);
  }

  @Override
  public double getDriveVoltage() {
    return driveSparkMax.getBusVoltage() * driveSparkMax.getAppliedOutput();
  }

  @Override
  public double getAbsoluteEncoderOffset() {
    return absoluteEncoderOffset;
  }
}
