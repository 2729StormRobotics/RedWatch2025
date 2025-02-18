package frc.robot.subsystems.arm;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;


import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
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


  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutVoltage m_appliedVoltage = Volts.mutable(0);
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutAngle m_angle = Radians.mutable(0);
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reall?ocation.
  private final MutAngularVelocity m_velocity = RotationsPerSecond.mutable(0);
  private SysIdRoutine SysId;

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

    checkAndResetABSEncoder();

    // Update the PID constants if they have changed
    if (logP.get() != io.getP())
      io.setP(logP.get());

    if (logI.get() != io.getI())
      io.setI(logI.get());

    if (logD.get() != io.getD())
      io.setD(logD.get());

    Logger.processInputs("Arm", inputs);

    SysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.per(Second).of(ArmConstants.RAMP_RATE),
                Volts.of(ArmConstants.STEP_VOLTAGE),
                null),
            new SysIdRoutine.Mechanism(
                v -> io.setVoltage(v.in(Volts)),
                (sysidLog) -> {
                  sysidLog
                      .motor("pivot")
                      .voltage(m_appliedVoltage.mut_replace(inputs.armAppliedVolts, Volts))
                      .angularPosition(m_angle.mut_replace(inputs.armPositionRad, Rotations))
                      .angularVelocity(
                          m_velocity.mut_replace(inputs.armVelocityRadPerSec, RotationsPerSecond));
                },
                this));
  }

  public void checkAndResetABSEncoder() {
    boolean isPressed = io.getHallEffect();
    if ((isPressed) && (getSide())){
      io.changeOffset((180-io.getArmAngleDegrees()));
    }
    if ((isPressed) && (!getSide())){
      io.changeOffset((0-io.getArmAngleDegrees()));
    }
  }
  public void setPosition(double position) {
    io.setArmPosition(position);
  }

  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  //true is right side, false is left
  private boolean getSide(){
    return io.getArmAngleDegrees() >= 90;
  }

  public void setSpeed(double speed){
    io.setSpeed(speed);
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
          return io.getArmAngleDegrees() < 5;
        },
        this);
  }
  public Command quasistaticForward() {
    return SysId.quasistatic(Direction.kForward)
        .until(() -> io.getArmAngleDegrees() > ArmConstants.ARM_MAX_ANGLE)
        .alongWith(
            new InstantCommand(
                () -> Logger.recordOutput("AlgaePivot/sysid-test-state-", "quasistatic-forward")));
  }

  public Command quasistaticBack() {
    return SysId.quasistatic(Direction.kReverse)
        .until(() -> io.getArmAngleDegrees() < ArmConstants.ARM_MIN_ANGLE)
        .alongWith(
            new InstantCommand(
                () -> Logger.recordOutput("AlgaePivot/sysid-test-state-", "quasistatic-reverse")));
  }

  public Command dynamicForward() {
    return SysId.dynamic(Direction.kForward)
        .until(() -> io.getArmAngleDegrees() > ArmConstants.ARM_MAX_ANGLE)
        .alongWith(
            new InstantCommand(
                () -> Logger.recordOutput("AlgaePivot/sysid-test-state-", "dynamic-forward")));
  }

  public Command dynamicBack() {
    return SysId.dynamic(Direction.kReverse)
        .until(() -> io.getArmAngleDegrees() < ArmConstants.ARM_MIN_ANGLE)
        .alongWith(
            new InstantCommand(
                () -> Logger.recordOutput("AlgaePivot/sysid-test-state-", "dynamic-reverse")));
  }
}