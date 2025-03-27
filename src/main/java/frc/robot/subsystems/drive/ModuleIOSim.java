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

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volt;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.drive.ModuleConstants.*;

import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;

/**
 * Physics sim implementation of module IO using MapleSim.
 */
public class ModuleIOSim implements ModuleIO {
  private static final double LOOP_PERIOD_SECS = 0.02;

  private final SwerveModuleSimulation moduleSimulation;
  private final SimulatedMotorController.GenericMotorController driveMotor;
  private final SimulatedMotorController.GenericMotorController turnMotor;

  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0.0, 0.13);
  private final PIDController driveFeedback = new PIDController(0.1, 0.0, 0.0);
  private final PIDController turnFeedback = new PIDController(10.0, 0.0, 0.0);

  public ModuleIOSim(SwerveModuleSimulation moduleSimulation) {
    this.moduleSimulation = moduleSimulation;

    this.driveMotor = moduleSimulation
        .useGenericMotorControllerForDrive()
        .withCurrentLimit(Amps.of(60));
    this.turnMotor = moduleSimulation
        .useGenericControllerForSteer()
        .withCurrentLimit(Amps.of(20));
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // MapleSim updates internally, we just read the values.

    inputs.drivePositionRad = moduleSimulation.getDriveWheelFinalPosition().in(Radians);
    inputs.drivePositionMeters = moduleSimulation.getDriveWheelFinalPosition().in(Radians) * kWheelDiameterMeters / 2;
    inputs.driveVelocityRadPerSec = moduleSimulation.getDriveWheelFinalSpeed().in(RadiansPerSecond);
    inputs.driveVelocityMeterPerSec =
        moduleSimulation.getDriveWheelFinalSpeed().in(RadiansPerSecond) * kWheelDiameterMeters / 2;
    inputs.driveAppliedVolts = driveAppliedVolts;
    // inputs.driveCurrentAmps = new double[] {Math.abs(driveMotor.().inAmps())};

    inputs.turnAbsolutePosition = moduleSimulation.getSteerAbsoluteFacing();
    // inputs.turnPosition = moduleSimulation.getSteerAbsoluteAngle().in(Radians);
    inputs.turnVelocityRadPerSec = moduleSimulation.getSteerAbsoluteEncoderSpeed().in(RadiansPerSecond);
    inputs.turnAppliedVolts = turnAppliedVolts;
    // inputs.turnCurrentAmps = new double[] {Math.abs(turnMotor.getCurrent().inAmps())};

    inputs.odometryTimestamps = new double[] {Timer.getFPGATimestamp()};
    inputs.odometryDrivePositionsRad = new double[] {inputs.drivePositionRad};
    inputs.odometryTurnPositions = new Rotation2d[] {inputs.turnPosition};
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    driveMotor.requestVoltage(Voltage.ofBaseUnits(driveAppliedVolts, Volts));
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    turnMotor.requestVoltage(Voltage.ofBaseUnits(turnAppliedVolts, Volts));
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    velocityRadPerSec *= 2 / kWheelDiameterMeters;
    setDriveVoltage(
        driveFeedforward.calculate(velocityRadPerSec)
            + driveFeedback.calculate(moduleSimulation.getDriveWheelFinalSpeed().in(RadiansPerSecond), velocityRadPerSec));
  }

  @Override
  public void setTurnPosition(double angle) {
    setTurnVoltage(turnFeedback.calculate(moduleSimulation.getSteerAbsoluteAngle().in(Radians), angle));
  }

  @Override
  public void setDrivePIDFF(double p, double i, double d, double ff) {
    driveFeedback.setPID(p, i, d);
    // driveFeedforward.setGain(ff);
  }

  @Override
  public void setTurnPIDFF(double p, double i, double d, double ff) {
    turnFeedback.setPID(p, i, d);
  }

  @Override
  public double getTurnPositionError(double angle) {
    return turnFeedback.getErrorDerivative();
  }
}