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

import static frc.robot.subsystems.elevator.ElevatorConstants.kDElevator;
import static frc.robot.subsystems.elevator.ElevatorConstants.kElevatorMaxOutputPower;
import static frc.robot.subsystems.elevator.ElevatorConstants.kElevatorMinOutputPower;
import static frc.robot.subsystems.elevator.ElevatorConstants.kIElevator;
import static frc.robot.subsystems.elevator.ElevatorConstants.kLeftElevatorCanId;
import static frc.robot.subsystems.elevator.ElevatorConstants.kPElevator;
import static frc.robot.subsystems.elevator.ElevatorConstants.kRightElevatorCanId;
import static frc.robot.subsystems.elevator.ElevatorConstants.offsetAdd;
import static frc.robot.subsystems.elevator.ElevatorConstants.offsetMult;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAnalogSensor;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Module IO implementation for SparkFlex elevator motor controller, SparkFlex
 * turn
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
public class ElevatorIOSparkFlex implements ElevatorIO {

  private final SparkFlex elevatorLeftSparkFlex;
  private final SparkFlex elevatorRightSparkFlex;

  private SparkFlexConfig elevatorConfig;
  private SparkFlexConfig elevatorConfigLeft;
  private SparkFlexConfig elevatorConfigRight = new SparkFlexConfig();

  private final double absoluteEncoderOffset = 0;

  public double pot_val = 0;
  public double elevator_height = 0;
  public double offset = 0;

  public final double VoltsToDistanceMeters = 8.4582;

  public final RelativeEncoder m_ElevatorLeftEncoder;
  public final RelativeEncoder m_ElevatorRightEncoder;

  private double kP = kPElevator;
  private double kI = kIElevator;
  private double kD = kDElevator;

  public ElevatorIOSparkFlex() {
    // Initialize motors
    elevatorLeftSparkFlex = new SparkFlex(kLeftElevatorCanId, MotorType.kBrushless);
    elevatorRightSparkFlex = new SparkFlex(kRightElevatorCanId, MotorType.kBrushless);

    // Configure motor controllers
    configureMotors();
    configureSoftLimits();
    configureLimitSwitches();

    // Initialize encoders
    m_ElevatorLeftEncoder = elevatorLeftSparkFlex.getEncoder();
    m_ElevatorRightEncoder = elevatorRightSparkFlex.getEncoder();

    // Set CAN timeout (consider setting a nonzero value)
    elevatorLeftSparkFlex.setCANTimeout(50);
    elevatorRightSparkFlex.setCANTimeout(50);

    // Enable follower mode
    elevatorConfigRight.follow(elevatorLeftSparkFlex, true);
    elevatorLeftSparkFlex.pauseFollowerMode();
    // Apply configurations
    elevatorLeftSparkFlex.configure(elevatorConfigLeft, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    elevatorRightSparkFlex.configure(elevatorConfigRight, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  // Separate method for configuring motors
  private void configureMotors() {
    elevatorConfig = new SparkFlexConfig();
    elevatorConfig.closedLoop.p(kPElevator);
    elevatorConfig.closedLoop.i(kIElevator);
    elevatorConfig.closedLoop.d(kDElevator);
    elevatorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    elevatorConfig.closedLoop.outputRange(kElevatorMinOutputPower, kElevatorMaxOutputPower);
  }

  // Separate method for configuring soft limits
  private void configureSoftLimits() {
    SoftLimitConfig softLimits = new SoftLimitConfig();
    softLimits.forwardSoftLimit(53);
    softLimits.forwardSoftLimitEnabled(true);
    softLimits.reverseSoftLimit(-1);
    softLimits.reverseSoftLimitEnabled(true);
    elevatorConfig.apply(softLimits);
  }

  // Separate method for configuring limit switches
  private void configureLimitSwitches() {
    elevatorConfigLeft = new SparkFlexConfig();
    elevatorConfigLeft.apply(elevatorConfig);
    elevatorConfigLeft.analogSensor.positionConversionFactor(VoltsToDistanceMeters);
    elevatorConfigLeft.limitSwitch.forwardLimitSwitchEnabled(true);
    elevatorConfigLeft.limitSwitch.reverseLimitSwitchEnabled(true);
    elevatorConfigLeft.limitSwitch.forwardLimitSwitchType(Type.kNormallyOpen);
    elevatorConfigLeft.limitSwitch.reverseLimitSwitchType(Type.kNormallyOpen);
  }

  public void setElevatorHeight(double targetHeight) {
    elevatorLeftSparkFlex.getClosedLoopController().setReference(targetHeight, SparkFlex.ControlType.kPosition);
  }

  public boolean isTopLimitTriggered() {
    return elevatorLeftSparkFlex.getForwardLimitSwitch().isPressed();
  }

  public boolean isBottomLimitTriggered() {
    return elevatorLeftSparkFlex.getReverseLimitSwitch().isPressed();
  }

  public double getPotVal() {
    return elevatorLeftSparkFlex.getAnalog().getPosition();
  }

  public SparkAnalogSensor StringPot() {
    return elevatorLeftSparkFlex.getAnalog();
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {

    double kWheelDiameterMeters = 1.0;

    inputs.elevatorPositionRad = m_ElevatorLeftEncoder.getPosition() / (kWheelDiameterMeters / 2);
    inputs.elevatorPositionMeters = m_ElevatorLeftEncoder.getPosition();
    inputs.elevatorVelocityMeterPerSec = m_ElevatorLeftEncoder.getVelocity();
    inputs.elevatorVelocityRadPerSec = m_ElevatorLeftEncoder.getVelocity() / (kWheelDiameterMeters / 2);
    inputs.elevatorAppliedVolts = elevatorLeftSparkFlex.getAppliedOutput() * elevatorLeftSparkFlex.getBusVoltage();
    inputs.elevatorCurrentAmps = new double[] { elevatorLeftSparkFlex.getOutputCurrent() };

    // turnPositionQueue.clear();
    SmartDashboard.putBoolean("Top Limit Switch Triggered?", isTopLimitTriggered());
    SmartDashboard.putBoolean("Bottom Limit Switch Triggered?", isBottomLimitTriggered());
    SmartDashboard.putNumber("Elevator Voltage: ",
        elevatorLeftSparkFlex.getBusVoltage() * elevatorLeftSparkFlex.getAppliedOutput());
    SmartDashboard.putNumber("Elevator Height: ", getElevatorHeight());
    SmartDashboard.putNumber("Velocity: ", m_ElevatorLeftEncoder.getVelocity());
    SmartDashboard.putNumber("Pot Val", getPotVal());
    SmartDashboard.putNumber("left encoder", getLeftEncoderDistance());
    SmartDashboard.putNumber("right encoder", getRightEncoderDistance());
  }

  public double getLeftEncoderDistance() {
    return m_ElevatorLeftEncoder.getPosition();
  }

  public double getRightEncoderDistance() {
    return m_ElevatorRightEncoder.getPosition();
  }

  @Override
  public void setElevatorVelocity(double velocityRadPerSec) {
    elevatorLeftSparkFlex.getClosedLoopController().setReference(velocityRadPerSec, SparkFlex.ControlType.kVelocity);
  }

  @Override
  public void setElevatorVoltage(double volts) {
    elevatorLeftSparkFlex.setVoltage(volts);
  }

  @Override
  public void setElevatorPower(double power) {

    SmartDashboard.putNumber("power sending", power);
    elevatorLeftSparkFlex.set(power);
  }

  @Override
  public double getElevatorVoltage() {
    return elevatorLeftSparkFlex.getBusVoltage() * elevatorLeftSparkFlex.getAppliedOutput();
  }

  @Override
  public double getAbsoluteEncoderOffset() {
    return absoluteEncoderOffset;
  }

  @Override
  public void stop() {
    setElevatorVoltage(0.0);
  }

  @Override
  public void setP(double p) {
    kP = p;
    elevatorConfigLeft.closedLoop.p(p);
  }

  @Override
  public void setI(double i) {
    kI = i;
    elevatorConfigLeft.closedLoop.i(i);
  }

  @Override
  public void setD(double d) {
    kD = d;
    elevatorConfigLeft.closedLoop.d(d);
  }

  @Override
  public double getP() {
    return kP;
  }

  @Override
  public double getI() {
    return kI;
  }

  @Override
  public double getD() {
    return kD;
  }

  public double getElevatorHeight() {

    return getPotVal() * offsetMult + offsetAdd;
  }

  // Periodically calculates the value of the pot.
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
