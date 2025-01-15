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
import static frc.robot.subsystems.elevator.ElevatorConstants.kStringPotPort;


import com.revrobotics.spark.SparkLowLevel.PeriodicFrame;
import com.revrobotics.spark.SparkMax;
import com.pathplanner.lib.config.ModuleConfig;
import com.revrobotics.AbsoluteEncoder;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.subsystems.drive.MotorConfigs;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorIOInputs;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.Timer;

import java.util.LinkedList;
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
public class ElevatorIOSparkFlex extends SubsystemBase {

  private final SparkFlex elevatorLeftSparkFlex;
  private final SparkFlex elevatorRightSparkFlex;

  elevatorPIDController.setP(kPElevator);
  elevatorPIDController.setI(kIElevator);
  +elevatorPIDController.setD(kDElevator);

  private SparkFlexConfig elevatorConfig;
  private final SparkClosedLoopController elevatorPIDController;

  private final RelativeEncoder elevatorEncoder;

  private final Queue<Double> timestampQueue;
  private final Queue<Double> elevatorPositionQueue;

  private final double absoluteEncoderOffset;
  public final AnalogPotentiometer elevatorPot = new AnalogPotentiometer(0); //idk change it later

  private DigitalInput m_ObjectDectector = new DigitalInput(ElevatorConstants.kStringPotPort);
  private SparkMax m_elevatorLeftSparkMotor = new SparkMax(ElevatorConstants.kLeftElevatorMotorPort, MotorType.kBrushless); 
  private SparkMax m_elevatorRightSparkMotor = new SparkMax(ElevatorConstants.kRightElevatorMotorPort, MotorType.kBrushless); 

  private boolean isOpen = false;
  private Timer timer;

  public final AnalogPotentiometer pot = new AnalogPotentiometer(1);
  public double pot_val;
  public double offset = 0;
    
  public ElevatorIOSparkFlex() {
    // need to complete
    elevatorLeftSparkFlex = new SparkFlex(kLeftElevatorCanId); 
    elevatorRightSparkFlex = new SparkFlex(kRightElevatorCanId);

    // configure motor controllers
    elevatorConfig = new SparkFlexConfig();
    elevatorLeftSparkFlex.configure(elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    elevatorRightSparkFlex.configure(elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // initialize encoders
    m_ElevatorLeftEncoder = elevatorLeftSparkFlex.getEncoder();
    m_ElevatorRightEncoder = elevatorRightSparkFlex.getEncoder();

    // initialize PID controller
    elevatorPIDController = elevatorLeftSparkFlex.getClosedLoopController();
    
    elevatorPot = new AnalogPotentiometer(0);
    m_ObjectDectector = new DigitalInput(kStringPotPort);
    timestampQueue = new LinkedList<>();
    elevatorPositionQueue = new LinkedList<>();
    timer = new Timer();

    }

  public final RelativeEncoder m_ElevatorLeftEncoder;
  m_LeftEncoder = elevatorLeftSparkFlex.getEncoder();

  public final RelativeEncoder m_ElevatorRightEncoder;
  m_RightEncoder = elevatorRightSparkFlex.getEncoder();
  
  elevatorSparkFlex.configure(MotorConfigs.drivingConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  
  elevatorSparkFlex.setCANTimeout(0);

  // elevatorEncoder = elevatorSparkFlex.getEncoder();
  
  elevatorPIDController = elevatorSparkFlex.getClosedLoopController();

  public void setTargetPosition(double position) {
    elevatorPIDController.setReference(position, SparkFlex.ControlType.kPosition);
    
  }
  
  public void setPower(double power) {
    elevatorLeftSparkFlex.set(power);
    elevatorRightSparkFlex.set(power);
  }

  // Log things?
  /*timestampQueue = SparkFlexOdometryThread.getInstance().makeTimestampQueue();
  elevatorPositionQueue = SparkFlexOdometryThread.getInstance().registerSignal(elevatorEncoder::getPosition);
  */


  // @Override
  // public void updateInputs(ElevatorIOInputs inputs) {
    
  //   double kWheelDiameterMeters = 0.0;

  //   inputs.elevatorPositionRad = elevatorEncoder.getPosition() / (kWheelDiameterMeters / 2);
  //   inputs.elevatorPositionMeters = elevatorEncoder.getPosition();
  //   inputs.elevatorVelocityMeterPerSec = elevatorEncoder.getVelocity();
  //   inputs.elevatorVelocityRadPerSec = elevatorEncoder.getVelocity() / (kWheelDiameterMeters / 2);
  //   inputs.elevatorAppliedVolts = elevatorSparkFlex.getAppliedOutput() * elevatorSparkFlex.getBusVoltage();
  //   inputs.elevatorCurrentAmps = new double[] { elevatorSparkFlex.getOutputCurrent() };

  //       //.mapToDouble((Double value) -> Units.rotationsToRadians(value))
  //       //.toArray();
  //       //.map((Double value) -> Rotation2d.fromRotations(value))
  //       //.toArray(Rotation2d[]::new);
  //   timestampQueue.clear();
  //   elevatorPositionQueue.clear();
  //   //turnPositionQueue.clear();
  // }

 
  // // Gets the distance from this value to this value. its tweakoing
  // public default double getDistance(){
  //   return pot_val;
  // }
  // public double getLeftEncoderDistance() {
  //   return -m_LeftEncoder.getPosition();
  // }

  // public double getRightEncoderDistance() {
  //   return -m_RightEncoder.getPosition();
  // } 


  // @Override
  // public void setElevatorVelocity(double velocityRadPerSec) {
  //   elevatorPIDController.setReference(velocityRadPerSec, SparkFlex.ControlType.kVelocity);
  // }

  // @Override
  // public void setElevatorVoltage(double volts) {
  //   elevatorSparkFlex.setVoltage(volts);
  // } 

  // @Override
  // public double getElevatorVoltage() {
  //   return elevatorSparkFlex.getBusVoltage() * elevatorSparkFlex.getAppliedOutput();
  // }

  // @Override
  // public double getAbsoluteEncoderOffset() {
  //   return absoluteEncoderOffset;
  // }
  // Periodically calculates the value of the pot.
  public void periodic() {
    // This method will be called once per scheduler run
    pot_val = (elevatorPot.get()*50)-offset;
    // pot_val = ((pot.get())*50);
  }
}

