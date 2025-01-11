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

package frc.robot.subsystems.elevator;

import static frc.robot.Constants.VisionConstants.*;
import static frc.robot.subsystems.elevator.ElevatorConstants.kPElevator;
import static frc.robot.subsystems.elevator.ElevatorConstants.kIElevator;
import static frc.robot.subsystems.elevator.ElevatorConstants.kDElevator;
import static frc.robot.subsystems.elevator.ElevatorConstants.kSElevator;
import static frc.robot.subsystems.elevator.ElevatorConstants.kGElevator;
import static frc.robot.subsystems.elevator.ElevatorConstants.kVElevator;
import static frc.robot.subsystems.elevator.ElevatorConstants.kAElevator;
import static frc.robot.subsystems.elevator.ElevatorConstants.kLeftElevatorCanId;
import static frc.robot.subsystems.elevator.ElevatorConstants.kRightElevatorCanId;

import com.revrobotics.spark.SparkLowLevel.PeriodicFrame;
import com.pathplanner.lib.config.ModuleConfig;
import com.revrobotics.AbsoluteEncoder;
import edu.wpi.first.wpilibj.AnalogPotentiometer;

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
import frc.robot.subsystems.drive.MotorConfigs;

import java.util.Queue;

/**
 * Module IO implementation for SparkMax elevator motor controller, SparkMax turn
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
 * motion on the elevator motor will propel the robot forward) and copy the
 * reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Elevator/ModuleX/TurnAbsolutePositionRad"
 */
public class ElevatorIOSparkMax implements ElevatorIO {

  private final SparkMax elevatorSparkMax;

  private SparkMaxConfig elevatorConfig;
  private final SparkClosedLoopController elevatorPIDController;

  private final RelativeEncoder elevatorEncoder;

  private final Queue<Double> timestampQueue;
  private final Queue<Double> elevatorPositionQueue;

  private final double absoluteEncoderOffset;

  public ElevatorIOSparkMax(int index) {
    switch (index) {
      case 0: // Front Left
        elevatorSparkMax = new SparkMax(kLeftElevatorCanId, MotorType.kBrushless);
        absoluteEncoderOffset = ElevatorConstants.kFrontLeftChassisAngularOffset; // MUST BE CALIBRATED/FIXED
        break;
      case 1: // Front Right
        elevatorSparkMax = new SparkMax(kRightElevatorCanId, MotorType.kBrushless);
        absoluteEncoderOffset = ElevatorConstants.kFrontRightChassisAngularOffset; // MUST BE CALIBRATED/FIXED
        break;
      default:
        throw new RuntimeException("Invalid module index");
    }
    elevatorSparkMax.configure(MotorConfigs.drivingConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    elevatorSparkMax.setCANTimeout(0);

    elevatorEncoder = elevatorSparkMax.getEncoder();
    
    elevatorPIDController = elevatorSparkMax.getClosedLoopController();

    // Log things?
    /*timestampQueue = SparkMaxOdometryThread.getInstance().makeTimestampQueue();
    elevatorPositionQueue = SparkMaxOdometryThread.getInstance().registerSignal(elevatorEncoder::getPosition);
    */
    }


  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    
    double kWheelDiameterMeters = 0.0;

        inputs.elevatorPositionRad = elevatorEncoder.getPosition() / (kWheelDiameterMeters / 2);
    inputs.elevatorPositionMeters = elevatorEncoder.getPosition();
    inputs.elevatorVelocityMeterPerSec = elevatorEncoder.getVelocity();
    inputs.elevatorVelocityRadPerSec = elevatorEncoder.getVelocity() / (kWheelDiameterMeters / 2);
    inputs.elevatorAppliedVolts = elevatorSparkMax.getAppliedOutput() * elevatorSparkMax.getBusVoltage();
    inputs.elevatorCurrentAmps = new double[] { elevatorSparkMax.getOutputCurrent() };

        //.mapToDouble((Double value) -> Units.rotationsToRadians(value))
        //.toArray();
        //.map((Double value) -> Rotation2d.fromRotations(value))
        //.toArray(Rotation2d[]::new);
    timestampQueue.clear();
    elevatorPositionQueue.clear();
    //turnPositionQueue.clear();
  }

  @Override
  public void setElevatorVelocity(double velocityRadPerSec) {
    elevatorPIDController.setReference(velocityRadPerSec, SparkMax.ControlType.kVelocity);
  }

  @Override
  public void setElevatorVoltage(double volts) {
    elevatorSparkMax.setVoltage(volts);
  }

  @Override
  public double getElevatorVoltage() {
    return elevatorSparkMax.getBusVoltage() * elevatorSparkMax.getAppliedOutput();
  }

  @Override
  public double getAbsoluteEncoderOffset() {
    return absoluteEncoderOffset;
  }
}
