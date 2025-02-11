package frc.robot.subsystems.arm;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;


/*
 * NEED TO MAKE LIGAMENTS and Mechanisms
 * as well as 
 * 
 */
public class Arm extends SubsystemBase {
  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  PowerDistribution m_PDH = new PowerDistribution(20,ModuleType.kRev);


  private LoggedNetworkNumber logP;
  private LoggedNetworkNumber logI;
  private LoggedNetworkNumber logD;

  private double setpoint = 0;

  public Arm(ArmIO io) {
    this.io = io;

    SmartDashboard.putData(getName(), this);

    logP = new LoggedNetworkNumber("/SmartDashboard/Arm/P", io.getP());
    logI = new LoggedNetworkNumber("/SmartDashboard/Arm/I", io.getI());
    logD = new LoggedNetworkNumber("/SmartDashboard/Arm/D", io.getD());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.recordOutput("Arm Position", inputs.armPositionDegrees);
    Logger.recordOutput("Arm Velocity", inputs.armVelocityRadPerSec);
    Logger.recordOutput("Arm Current", inputs.armAppliedVolts);
    SmartDashboard.putNumber("Arm Position", inputs.armPositionDegrees);
    SmartDashboard.putNumber("Arm Velocity", inputs.armVelocityRadPerSec);
    SmartDashboard.putNumber("Arm Current", inputs.armAppliedVolts);
    SmartDashboard.putData("PDH", m_PDH);


    // Update the PID constants if they have changed
    if (logP.get() != io.getP())
      io.setP(logP.get());

    if (logI.get() != io.getI())
      io.setI(logI.get());

    if (logD.get() != io.getD())
      io.setD(logD.get());

    Logger.processInputs("Arm", inputs);
  }

  public void setPosition(double position) {
    io.setArmPosition(position);
  }

  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  //true is right side, false is left
  public boolean getSide(){
    return io.getArmAngleDegrees() >= 90;
  }

  public void setSpeed(double speed){
    if( !((speed > 0 && getSide() && io.getHallEffect()) || (speed < 0 && !getSide() && io.getHallEffect())) )
    {io.setSpeed(speed);}
  }

  public double getPosition() {
    return inputs.armPositionDegrees;
  }

  public double getVelocity() {
    return io.getArmVelocity();
  }

  public boolean atSetpoint() {
    return Math.abs(io.getArmAngleDegrees() - setpoint) < ArmConstants.PID_TOLERANCE
        && Math.abs(getVelocity()) < ArmConstants.PID_VELOCITY_TOLERANCE;
  }

  public void runPID() {
    io.setArmPosition(setpoint);
  }

  public void setPID(double setpoint) {
    this.setpoint = setpoint;
    Logger.recordOutput("Arm/Setpoint", setpoint);
  }


  public void addPID(double setpointAdd) {
    this.setpoint += setpointAdd;
    this.setpoint = MathUtil.clamp(
        this.setpoint,
        0,
        180);

    Logger.recordOutput("Arm/Setpoint", setpoint);
  }

  public Command PIDCommand(double setpoint) {
    return new FunctionalCommand(
        () -> setPID(setpoint), () -> runPID(), (stop) -> setVoltage(0), this::atSetpoint, this);
  }

  public Command PIDCommandForever(DoubleSupplier setpointSupplier) {
    return new FunctionalCommand(
        () -> setPID(setpointSupplier.getAsDouble()),
        () -> {
          setPID(setpointSupplier.getAsDouble());
          runPID();
        },
        (stop) -> setVoltage(0),
        () -> false,
        this);
  }

  public Command CalibrateArm(){
    return new FunctionalCommand(
      () -> {}, 
      () -> {setSpeed(0.1);},
      (stop) -> {
        io.changeOffset(-io.getArmAngleDegrees());
        io.stopArm();
      }, 
       () -> !getSide(),
       this);
  }

  public Command PIDCommandForever(double setpoint) {
    return new FunctionalCommand(
        () -> setPID(setpoint), () -> runPID(), (stop) -> setVoltage(0), () -> false, this);
  }

  public Command PIDHoldCommand() {
    return new FunctionalCommand(
        () -> setPID(io.getArmAngleDegrees()),
        () -> {
        },
        (stop) -> setVoltage(0),
        () -> false,
        this);
  }

  public Command PIDCommand(DoubleSupplier setpointSupplier) {
    return new FunctionalCommand(
        () -> setPID(setpointSupplier.getAsDouble()),
        () -> {
          Logger.recordOutput("ArmAngle", setpointSupplier.getAsDouble());
          setPID(setpointSupplier.getAsDouble());
          runPID();
        },
        (stop) -> setVoltage(0),
        this::atSetpoint,
        this);
  }

  // Allows manual control of the pivot arm for PID tuning
  public Command ManualCommand(DoubleSupplier speedSupplier) {
    return new FunctionalCommand(
        () -> setSpeed(speedSupplier.getAsDouble()),
        () -> setSpeed(speedSupplier.getAsDouble()),
        (stop) -> setSpeed(0),
        () -> false,
        this);
  }

  public Command stop() {
    return new FunctionalCommand(
        () -> { }, () -> io.setVoltage(0), (stop) -> io.stopArm(), () -> false, this);
  }

  public Command bringDownCommand() {
    return new FunctionalCommand(
        () -> {
        },
        () -> {
          setVoltage(-1);
          setpoint = 0;
        },
        (interrupted) -> {
          setVoltage(0);
        },
        () -> {
          return io.getArmAngleDegrees() < 1;
        },
        this);
  }
}