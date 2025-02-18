package frc.robot.subsystems.arm;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import java.util.Dictionary;

import com.revrobotics.AbsoluteEncoder;

import frc.robot.subsystems.arm.ArmIO.ArmIOInputs;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.arm.ArmConstants;

public class ArmIOSparkMax implements ArmIO {
    public SparkMax armSparkMaxLeft;
    public SparkMax armSparkMaxRight;
    public SparkMaxConfig armConfigRight;
    public SparkMaxConfig armConfigLeft;
    public SparkLimitSwitch hallEffect;
    public AbsoluteEncoder armAbsoluteEncoder;
    public double armEncoderOffset;

    private double kP = ArmConstants.kPArm;
    private double kI = ArmConstants.kIArm;
    private double kD = ArmConstants.kDArm;

    /*
     * This subsystem uses 2 motors to rotate our pivot arm from 0 - 180 degrees
     * Inputs are given in Degrees
     * 
     */
    public ArmIOSparkMax() {
        // Define motor
        armSparkMaxLeft = new SparkMax(ArmConstants.kArmCANID, MotorType.kBrushless);
        armSparkMaxRight = new SparkMax(ArmConstants.kArmCANID2, MotorType.kBrushless);

        // Define Configs for Hanger Motor

        armConfigRight = new SparkMaxConfig();
        armConfigRight.idleMode(IdleMode.kCoast);
        armConfigRight.limitSwitch.reverseLimitSwitchEnabled(false);

        // armConfigRight.softLimit.forwardSoftLimit(0);
        // armConfigRight.softLimit.forwardSoftLimitEnabled(true);
        // armConfigRight.softLimit.reverseSoftLimit(180);
        // armConfigRight.softLimit.reverseSoftLimitEnabled(true);


        
        armConfigLeft = new SparkMaxConfig();
        armConfigLeft.apply(armConfigRight);
        armConfigLeft.closedLoop.pid(kP, kI, kD);
        armConfigLeft.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        armConfigLeft.closedLoop.outputRange(ArmConstants.kArmMinOutputPower, ArmConstants.kArmMaxOutputPower);
        armConfigLeft.absoluteEncoder.velocityConversionFactor(6);
        armConfigLeft.absoluteEncoder.positionConversionFactor(360);

        armConfigRight.follow(armSparkMaxLeft, true);

        // burn motor
        armSparkMaxLeft.configure(armConfigLeft, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        armSparkMaxRight.configure(armConfigRight, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        // armSparkMaxRight.pauseFollowerMode();

        hallEffect = armSparkMaxRight.getReverseLimitSwitch();
        armAbsoluteEncoder = armSparkMaxLeft.getAbsoluteEncoder();
    }

    public void updateInputs(ArmIOInputs inputs) {
        // inputs.armCLC = armConfigRight.closedLoop;
        inputs.armAppliedVolts = (armSparkMaxRight.getBusVoltage() * armSparkMaxRight.getAppliedOutput());
        inputs.armPositionDegrees = getArmAngleDegrees();
        inputs.armVelocityRadPerSec = getArmVelocity();
    }

    @Override
    public void pivotClockwise() {

        armSparkMaxLeft.set(ArmConstants.kArmMotorSpeed);
    }

    @Override
    public boolean getHallEffect(){
        return hallEffect.isPressed();
    }

    @Override
    public void changeOffset(double newOffset){
        armEncoderOffset += newOffset;
    }   

    @Override
    public double getVoltage() {
        return armSparkMaxRight.getBusVoltage();
    }

    // @Override
    // public double getArmAngleRad() {
    // return armAbsoluteEncoder.getPosition();
    // // this is not right
    // }

    @Override
    public double getArmAngleDegrees() {
        return armAbsoluteEncoder.getPosition() + armEncoderOffset;
        // this is not righth
    }

    @Override
    public double getArmVelocity() {
        return armAbsoluteEncoder.getVelocity();
    }

    @Override
    public void pivotCounterclockwise() {

        armSparkMaxLeft.set(-ArmConstants.kArmMotorSpeed);
    };

    @Override
    public void setVoltage(double voltage) {
        armSparkMaxLeft.setVoltage(voltage);
    }

    
    @Override
    public void setSpeed(double speed) {
        armSparkMaxLeft.set(speed);
    }

    @Override
    public void setArmPosition(double kAngle) {
        armSparkMaxLeft.getClosedLoopController().setReference(kAngle, ControlType.kPosition);
    }

    @Override
    public void stopArm() {
        armSparkMaxLeft.stopMotor();
    };

    private void updateMotorConfig(SparkFlexConfig config) {
        // DO NOT RESET paramaters becasue we only want to change some paramaters, not
        // all
        // DO NOT PERSIST because this is a temporary change that we don't want to save
        // to memory
        armSparkMaxLeft.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        armSparkMaxRight.configure(
                config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
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
}