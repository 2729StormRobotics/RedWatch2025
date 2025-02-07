package frc.robot.subsystems.elevator;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkAnalogSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;

import frc.robot.Constants;

public class ElevatorIOSparkFlex implements ElevatorIO {
  private SparkFlex leftMotor;
  // right follows left
  private SparkFlex rightMotor;
  private SparkClosedLoopController leftController;
  private AbsoluteEncoder leftEncoder;
  private SparkAnalogSensor string_potentiometer;

  // used to track the target setpoint of the robot
  private double setpoint = 0;

  // Because SparkFlex does not have getters for pid, use these variables to keep
  // track of dynamic
  // pid values
  private double kP = ElevatorConstants.kElevatorRealPID[0];
  private double kI = ElevatorConstants.kElevatorRealPID[1];
  private double kD = ElevatorConstants.kElevatorRealPID[2];
  private double kFF = ElevatorConstants.kElevatorRealPID[3];

  public ElevatorIOSparkFlex() {
    leftMotor = new SparkFlex(ElevatorConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
    rightMotor = new SparkFlex(ElevatorConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);

    leftController = leftMotor.getClosedLoopController();
    string_potentiometer = leftMotor.getAnalog();
    leftEncoder = leftMotor.getAbsoluteEncoder();

    SparkFlexConfig leftConfig = new SparkFlexConfig();
    leftConfig.smartCurrentLimit(Constants.NEO_CURRENT_LIMIT).idleMode(IdleMode.kCoast);
    leftConfig.closedLoop
        .pidf(kP, kI, kD, kFF)
        .feedbackSensor(FeedbackSensor.kAnalogSensor);
    leftConfig.encoder
        .positionConversionFactor(ElevatorConstants.POSITION_CONVERSION_FACTOR)
        .velocityConversionFactor(ElevatorConstants.POSITION_CONVERSION_FACTOR / 60.0);
    leftConfig.analogSensor
        .positionConversionFactor(ElevatorConstants.STRINGPOT_POSITION_CONVERSION_FACTOR)
        .velocityConversionFactor(1.0);

    leftConfig.limitSwitch
        .forwardLimitSwitchEnabled(true)
        .forwardLimitSwitchType(Type.kNormallyOpen)
        .reverseLimitSwitchEnabled(true)
        .reverseLimitSwitchType(Type.kNormallyOpen);
    SparkFlexConfig rightConfig = new SparkFlexConfig();
    rightConfig.apply(leftConfig);
    rightConfig.follow(leftMotor, true);

    // reset safe kResetSafeParameters switches the motor to default paramaters,
    // then adds the
    // changes from the config object
    // persist paramaters saves these changes to the motor memory so it doesn't get
    // cooked during
    // brownouts
    // only use persist in the BEGINNING, not later
    leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightMotor.configure(
        rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    leftMotor.pauseFollowerMode();
  }

  private void updateMotorConfig(SparkFlexConfig config) {
    // DO NOT RESET paramaters becasue we only want to change some paramaters, not
    // all
    // DO NOT PERSIST because this is a temporary change that we don't want to save
    // to memory
    leftMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    rightMotor.configure(
        config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.setpointMeters = setpoint;
    inputs.positionMeters = getPosition();
    inputs.velocityMetersPerSec = getVelocity();
    inputs.appliedVoltage = leftMotor.getAppliedOutput() * leftMotor.getBusVoltage();
    inputs.motorCurrent = new double[] { leftMotor.getOutputCurrent(), rightMotor.getOutputCurrent() };
    inputs.motorTemperature = new double[] { leftMotor.getMotorTemperature(), rightMotor.getMotorTemperature() };
    SmartDashboard.putNumber("StringPot/Position", getStringPot().getPosition());
    SmartDashboard.putNumber("StringPot/Velocity", getStringPot().getVelocity());
    SmartDashboard.putNumber("StringPot/Voltage", getStringPot().getVoltage());
  }

  @Override
  public double getPosition() {
    // get the absolute position in radians, then convert to meters
    return stringPotToElevator(getStringPot().getPosition());
  }

  public SparkAnalogSensor getStringPot() {
    return leftMotor.getAnalog();
  }

  private double elevatorToStringPot(double elevatorHeight) {
    return 0.0;
  }

  private double stringPotToElevator(double stringLength) {
    return stringLength * ElevatorConstants.stringPottoElevatorConversion + ElevatorConstants.ElevatorOffset;
  }

  @Override
  public void goToSetpoint(double setpoint) {
    this.setpoint = setpoint;
    leftController.setReference(elevatorToStringPot(setpoint), ControlType.kPosition);
  }

  @Override
  public boolean atSetpoint() {
    // if the difference between setpoint and position is less than the tolerance
    return (Math.abs(getSetpoint() - getPosition()) < ElevatorConstants.SETPOINT_TOLERANCE_METERS);
  }

  @Override
  public double getVelocity() {
    return leftEncoder.getVelocity();
  }

  @Override
  public void setVelocity(double velocity) {
    leftMotor.set(velocity);
  }

  @Override
  public void setBrakeMode(boolean brakeEnabled) {
    SparkFlexConfig config = new SparkFlexConfig();
    config.idleMode(brakeEnabled ? IdleMode.kBrake : IdleMode.kCoast);
    updateMotorConfig(config);
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

  @Override
  public double getFF() {
    return kFF;
  }

  @Override
  public void setP(double kP) {
    this.kP = kP;
    SparkFlexConfig config = new SparkFlexConfig();
    config.closedLoop.p(kP);
    updateMotorConfig(config);
  }

  @Override
  public void setI(double kI) {
    this.kI = kI;
    SparkFlexConfig config = new SparkFlexConfig();
    config.closedLoop.i(kI);
    updateMotorConfig(config);
  }

  @Override
  public void setD(double kD) {
    this.kD = kD;
    SparkFlexConfig config = new SparkFlexConfig();
    config.closedLoop.d(kD);
    updateMotorConfig(config);
  }

  @Override
  public void setFF(double kFF) {
    this.kFF = kFF;
    SparkFlexConfig config = new SparkFlexConfig();
    config.closedLoop.velocityFF(kFF);
    updateMotorConfig(config);
  }
}