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

  /**
   * Checks and resets the absolute encoder based on the hall effect sensor.
   */
  public void checkAndResetABSEncoder() {
    boolean isPressed = io.getHallEffect();
    if ((isPressed) && (getSide())){
      io.changeOffset((180-io.getArmAngleDegrees()));
    }
    if ((isPressed) && (!getSide())){
      io.changeOffset((0-io.getArmAngleDegrees()));
    }
  }
  /**
   * Sets the target position of the arm.
   * @param position The position to set the arm to.
   */
  public void setPosition(double position) {
    io.setArmPosition(position);
  }

  /**
   * Sets the voltage applied to the arm motor.
   * @param voltage The voltage to apply.
   */
  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  /**
   * Gets the side of the arm.
   * @return true if the arm is on the right side, false if on the left.
   */
  //true is right side, false is left
  private boolean getSide(){
    return io.getArmAngleDegrees() >= 90;
  }

  /**
    * Sets the speed of the arm.
    * @param speed The speed to set the arm to.
    */
  public void setSpeed(double speed){
    io.setSpeed(speed);
  }

  /**
   * Gets the current position of the arm.
   * @return The current position of the arm.
   */
  public double getPosition() {
    return inputs.armPositionDegrees;
  }

  /**
   * Gets the current velocity of the arm.
   * @return The current velocity of the arm.
   */
  public double getVelocity() {
    return io.getArmVelocity();
  }

  /**
   * Checks if the arm is at the setpoint.
   * @return true if the arm is at the setpoint, false otherwise.
   */
  public boolean atSetpoint() {
    return Math.abs(io.getArmAngleDegrees() - setpoint) < ArmConstants.PID_TOLERANCE;
  }

  /**
   * Runs the PID control loop to move the arm to the setpoint.
   */
  public void runPID() {
    io.setArmPosition(setpoint);
  }

  /**
   * Sets the setpoint for the PID control loop.
   * @param setpoint The setpoint to set.
   */
  public void setPID(double setpoint) {
    this.setpoint = setpoint;
    Logger.recordOutput("Arm/Setpoint", setpoint);
  }

  /**
   * Adds to the current setpoint, clamping the result within the arm's limits.
   * @param setpointAdd The value to add to the setpoint.
   */
  public void addPID(double setpointAdd) {
    this.setpoint += setpointAdd;
    this.setpoint = MathUtil.clamp(
        this.setpoint,
        0,
        180);

    Logger.recordOutput("Arm/Setpoint", setpoint);
  }

  /**
   * Creates a command to move the arm to a specific position using PID control.
   * @param setpoint The target position for the arm.
   * @return A Command that moves the arm to the setpoint.
   */
  public Command PIDCommand(double setpoint) {
    return new FunctionalCommand(
        () -> setPID(setpoint), () -> runPID(), (stop) -> setVoltage(0), this::atSetpoint, this);
  }

  /**
   * Creates a command to continuously move the arm to a setpoint provided by a supplier.
   * @param setpointSupplier A supplier that provides the target position for the arm.
   * @return A Command that continuously updates the arm's target position.
   */
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

  /**
    * Command to calibrate the arm.
    * @return returns a command to calibrate the arm
    */
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

  /**
   * Creates a command to continuously move the arm to a fixed setpoint.
   * @param setpoint The target position for the arm.
   * @return A Command that continuously moves the arm to the setpoint.
   */
  public Command PIDCommandForever(double setpoint) {
    return new FunctionalCommand(
        () -> setPID(setpoint), () -> runPID(), (stop) -> setVoltage(0), () -> false, this);
  }

  /**
   * Creates a command to hold the current arm position using PID control.
   * @return A Command that holds the arm's current position.
   */
  public Command PIDHoldCommand() {
    return new FunctionalCommand(
        () -> setPID(io.getArmAngleDegrees()),
        () -> {
        },
        (stop) -> setVoltage(0),
        () -> false,
        this);
  }

  /**
   * Creates a command to move the arm to a setpoint provided by a supplier, finishing when the setpoint is reached.
   * @param setpointSupplier A supplier that provides the target position.
   * @return A Command that moves the arm to the provided setpoint.
   */
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

  /**
   * Creates a command for manual control of the arm for PID tuning.
   * @param speedSupplier A supplier that provides the arm speed.
   * @return A Command that allows manual control of the arm speed.
   */
  public Command ManualCommand(DoubleSupplier speedSupplier) {
    return new FunctionalCommand(
        () -> setSpeed(speedSupplier.getAsDouble()),
        () -> setSpeed(speedSupplier.getAsDouble()),
        (stop) -> setSpeed(0),
        () -> false,
        this);
  }

  /**
   * Creates a command to stop the arm motor.
   * @return A Command that stops the arm.
   */
  public Command stop() {
    return new FunctionalCommand(
        () -> { }, () -> io.setVoltage(0), (stop) -> io.stopArm(), () -> false, this);
  }

  /**
   * Creates a command to bring the arm down to a position below 5 degrees.
   *
   * @return A command to bring the arm down
   */
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
  /**
   * Creates a command to run quasistatic system identification in the forward direction.

   * @return A Command that runs quasistatic system identification forward.
   */
  public Command quasistaticForward() {
    return SysId.quasistatic(Direction.kForward)
        .until(() -> io.getArmAngleDegrees() > ArmConstants.ARM_MAX_ANGLE)
        .alongWith(
            new InstantCommand(
                () -> Logger.recordOutput("AlgaePivot/sysid-test-state-", "quasistatic-forward")));
  }

  /**
   * Creates a command to run quasistatic system identification in the reverse direction.
   * @return A Command that runs quasistatic system identification in reverse.
   */
  public Command quasistaticBack() {
    return SysId.quasistatic(Direction.kReverse)
        .until(() -> io.getArmAngleDegrees() < ArmConstants.ARM_MIN_ANGLE)
        .alongWith(
            new InstantCommand(
                () -> Logger.recordOutput("AlgaePivot/sysid-test-state-", "quasistatic-reverse")));
  }

  /**
   * Creates a command to run dynamic system identification in the forward direction.
   * @return A Command that runs dynamic system identification forward.
   */
  public Command dynamicForward() {
    return SysId.dynamic(Direction.kForward)
        .until(() -> io.getArmAngleDegrees() > ArmConstants.ARM_MAX_ANGLE)
        .alongWith(
            new InstantCommand(
                () -> Logger.recordOutput("AlgaePivot/sysid-test-state-", "dynamic-forward")));
  }

  /**
   * Creates a command to run dynamic system identification in the reverse direction.
   * @return A Command that runs dynamic system identification in reverse.
   */
  public Command dynamicBack() {
    return SysId.dynamic(Direction.kReverse)
        .until(() -> io.getArmAngleDegrees() < ArmConstants.ARM_MIN_ANGLE)
        .alongWith(
            new InstantCommand(
                () -> Logger.recordOutput("AlgaePivot/sysid-test-state-", "dynamic-reverse")));
  }
}
