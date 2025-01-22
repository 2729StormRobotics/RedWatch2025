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

import static frc.robot.subsystems.elevator.ElevatorConstants.kPElevator;
import static frc.robot.subsystems.elevator.ElevatorConstants.kIElevator;
import static frc.robot.subsystems.elevator.ElevatorConstants.kDElevator;
import static frc.robot.subsystems.elevator.ElevatorConstants.kLeftElevatorCanId;
import static frc.robot.subsystems.elevator.ElevatorConstants.kRightElevatorCanId;
import static frc.robot.subsystems.elevator.ElevatorConstants.kStringPotPort;


import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkAnalogSensor;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import java.util.Queue;

/**
 * Module IO implementation for SparkFlex elevator motor controller, SparkFlex turn
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
  private SparkFlexConfig elevatorConfigRight;
  // private final SparkClosedLoopController elevatorPIDController;

  // private final RelativeEncoder elevatorEncoder;


  private final double absoluteEncoderOffset = 0;
  public final AnalogPotentiometer elevatorPot = new AnalogPotentiometer(0); //idk change it later
  

  public final SparkAnalogSensor pot;
  public double pot_val;
  public double offset = 0;

  public final double VoltsToDistanceMeters = 1;
  
  public final RelativeEncoder m_ElevatorLeftEncoder;
  public final RelativeEncoder m_ElevatorRightEncoder;
  
  public ElevatorIOSparkFlex() {
    // need to complete
    elevatorLeftSparkFlex = new SparkFlex(kLeftElevatorCanId, MotorType.kBrushless); 
    elevatorRightSparkFlex = new SparkFlex(kRightElevatorCanId, MotorType.kBrushless);

    // configure motor controllers
    elevatorConfig = new SparkFlexConfig();
    elevatorConfig.closedLoop.p(kPElevator);
    elevatorConfig.closedLoop.i(kIElevator);
    elevatorConfig.closedLoop.d(kDElevator);  
    // left individual config
    elevatorConfigLeft = elevatorConfig;
    elevatorConfigLeft.analogSensor.positionConversionFactor(VoltsToDistanceMeters);
    elevatorConfigLeft.limitSwitch.forwardLimitSwitchEnabled(true);
    elevatorConfigLeft.limitSwitch.reverseLimitSwitchEnabled(true);
    elevatorConfigLeft.limitSwitch.forwardLimitSwitchType(Type.kNormallyOpen);
    elevatorConfigLeft.limitSwitch.reverseLimitSwitchType(Type.kNormallyOpen);

    pot = elevatorLeftSparkFlex.getAnalog();

    // Rigt individual config
    elevatorConfigRight = elevatorConfig;
    elevatorConfigRight.follow(elevatorLeftSparkFlex, true);


    // initialize encoders
    m_ElevatorLeftEncoder = elevatorLeftSparkFlex.getEncoder();
    m_ElevatorRightEncoder = elevatorRightSparkFlex.getEncoder();

    elevatorLeftSparkFlex.setCANTimeout(0);
    elevatorRightSparkFlex.setCANTimeout(0);

    // initialize PID controller
    
    // timestampQueue = new LinkedList<>();
    // elevatorPositionQueue = new LinkedList<>();


    elevatorLeftSparkFlex.configure(elevatorConfigLeft, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    elevatorRightSparkFlex.configure(elevatorConfigRight, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    }


  public void setElevatorHeight(double targetHeight, ElevatorIOInputs inputs) {
    elevatorLeftSparkFlex.getClosedLoopController().setReference(targetHeight, SparkFlex.ControlType.kPosition);
  }
  
  public void setLeftPower(double power) {
    elevatorLeftSparkFlex.set(power);
  }

  public void setRightPower(double power) {
    elevatorRightSparkFlex.set(power);
  }

  public boolean isTopLimitTriggered() {
    return elevatorLeftSparkFlex.getForwardLimitSwitch().isPressed();
  } 
  public boolean isBottomLimitTriggered() {
    return elevatorLeftSparkFlex.getReverseLimitSwitch().isPressed();
  } 

  public double getElevatorHeight() {
    return elevatorLeftSparkFlex.getAnalog().getPosition();
  }

  public SparkAnalogSensor StringPot() {
    return elevatorLeftSparkFlex.getAnalog();
  }
  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    
    double kWheelDiameterMeters = 0.0;

    inputs.elevatorPositionRad = m_ElevatorLeftEncoder.getPosition() / (kWheelDiameterMeters / 2);
    inputs.elevatorPositionMeters = m_ElevatorLeftEncoder.getPosition();
    inputs.elevatorVelocityMeterPerSec = m_ElevatorLeftEncoder.getVelocity();
    inputs.elevatorVelocityRadPerSec = m_ElevatorLeftEncoder.getVelocity() / (kWheelDiameterMeters / 2);
    inputs.elevatorAppliedVolts = elevatorLeftSparkFlex.getAppliedOutput() * elevatorLeftSparkFlex.getBusVoltage();
    inputs.elevatorCurrentAmps = new double[] { elevatorLeftSparkFlex.getOutputCurrent() };
    
    // turnPositionQueue.clear();
  }
 
  // Gets the distance from this value to this value. its tweakoing
  public double getDistance(){
    return pot_val;
  }
  public double getLeftEncoderDistance() {
    return -m_ElevatorLeftEncoder.getPosition();
  }

  public double getRightEncoderDistance() {
    return -m_ElevatorRightEncoder.getPosition();
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
  public double getElevatorVoltage() {
    return elevatorLeftSparkFlex.getBusVoltage() * elevatorLeftSparkFlex.getAppliedOutput();
  }

  @Override
  public double getAbsoluteEncoderOffset() {
    return absoluteEncoderOffset;
  }
  // Periodically calculates the value of the pot.
  public void periodic() {
    // This method will be called once per scheduler run
    pot_val = (elevatorPot.get()*50)-offset;
    // pot_val = ((pot.get())*50);
    SmartDashboard.putBoolean("Top Limit Switch Triggered?", isTopLimitTriggered());
    SmartDashboard.putBoolean("Bottom Limit Switch Triggered?", isBottomLimitTriggered());
    SmartDashboard.putNumber("Elevator Voltage: ", elevatorLeftSparkFlex.getBusVoltage() * elevatorLeftSparkFlex.getAppliedOutput());
    SmartDashboard.putNumber("Elevator Height: ", elevatorLeftSparkFlex.getAnalog().getPosition());
    SmartDashboard.putNumber("Velocity: ", m_ElevatorLeftEncoder.getVelocity());
  }
}

