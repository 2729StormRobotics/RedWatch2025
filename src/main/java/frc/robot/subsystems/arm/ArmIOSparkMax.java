package frc.robot.subsystems.arm;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
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

import java.time.Instant;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

import frc.robot.subsystems.arm.ArmIO.ArmIOInputs;
import frc.robot.subsystems.arm.ArmIOConstants;

public class ArmIOSparkMax implements ArmIO {
    public SparkMax armSparkMax1;
    public SparkMax armSparkMax2;
    public SparkMaxConfig armSparkMaxConfig;
    public SparkLimitSwitch metalDetector;
    public AbsoluteEncoder armAbsoluteEncoder;

    public ArmIOSparkMax() {
        // Define motor
        armSparkMax1 = new SparkMax(ArmIOConstants.kArmCANID, MotorType.kBrushless);
        armSparkMax2 = new SparkMax(ArmIOConstants.kArmCANID2, MotorType.kBrushless);

        // Define Configs for Hanger Motor

        armSparkMaxConfig = new SparkMaxConfig();
        armSparkMaxConfig.closedLoop.pid(ArmIOConstants.kPArm, ArmIOConstants.kIArm, ArmIOConstants.kDArm);
        armSparkMaxConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        armSparkMaxConfig.closedLoop.outputRange(ArmIOConstants.kArmMinOutputPower, ArmIOConstants.kArmMaxOutputPower);
        armSparkMaxConfig.idleMode(IdleMode.kBrake);

        armSparkMaxConfig.absoluteEncoder.velocityConversionFactor(0.10472);

        // burn motor
        armSparkMax1.configure(armSparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        armSparkMax2.configure(armSparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        metalDetector = armSparkMax1.getForwardLimitSwitch();
        armAbsoluteEncoder = armSparkMax2.getAbsoluteEncoder();
    }

    public void updateInputs(ArmIOInputs inputs) {
        ArmIO.ArmIOInputs.armCLC = armSparkMaxConfig.closedLoop;
        ArmIO.ArmIOInputs.armAppliedVolts = (armSparkMax1.getBusVoltage() * armSparkMax1.getAppliedOutput()); // divided
                                                                                                              // by some
                                                                                                              // number?
        ArmIO.ArmIOInputs.armPositionRad = getArmAngleRad();
        ArmIO.ArmIOInputs.armPositionDegrees = getArmAngleDegrees();
        ArmIO.ArmIOInputs.armVelocityRadPerSec = getArmVelocity();
    }

    @Override
    public void pivotClockwise() {
        armSparkMax1.set(ArmIOConstants.kArmMotorSpeed);
        armSparkMax2.set(ArmIOConstants.kArmMotorSpeed);
    }

    @Override
    public double getVoltage() {
        return armSparkMax1.getBusVoltage();
    }

    @Override
    public double getArmAngleRad() {
        return armAbsoluteEncoder.getPosition();
        // this is not right
    }

    @Override
    public double getArmAngleDegrees() {
        return armAbsoluteEncoder.getPosition() * (180 / Math.PI);
        // this is not righth
    }

    @Override
    public double getArmVelocity() {
        return armAbsoluteEncoder.getVelocity();
    }

    @Override
    public void pivotCounterclockwise() {
        armSparkMax1.set(-ArmIOConstants.kArmMotorSpeed);
        armSparkMax2.set(-ArmIOConstants.kArmMotorSpeed);
    };

    @Override
    public void setVoltage(double voltage) {
        armSparkMax1.setVoltage(voltage);
        armSparkMax2.setVoltage(voltage);
    }

    public void goAngle(double kAngle) {
        armSparkMax1.getClosedLoopController().setReference(kAngle, ControlType.kPosition);
        armSparkMax2.getClosedLoopController().setReference(kAngle, ControlType.kPosition);
    }

    @Override
    public void stopArm() {
        armSparkMax1.stopMotor();
        armSparkMax2.stopMotor();
    };

    @Override
    public void setP(double p) {
        ArmIOConstants.kPArm = p;
        armSparkMaxConfig.closedLoop.p(p);
    }

    @Override
    public void setI(double i) {
        ArmIOConstants.kIArm = i;
        armSparkMaxConfig.closedLoop.i(i);
    }

    @Override
    public void setD(double d) {
        ArmIOConstants.kDArm = d;
        armSparkMaxConfig.closedLoop.d(d);
    }

    @Override
    public double getP() {
        return ArmIOConstants.kPArm;
    }

    @Override
    public double getI() {
        return ArmIOConstants.kIArm;
    }

    @Override
    public double getD() {
        return ArmIOConstants.kDArm;
    }

    @Override
    public SequentialCommandGroup clockwise() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    this.pivotClockwise();
                }));
    };

    @Override
    public SequentialCommandGroup counterClockwise() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    this.pivotCounterclockwise();
                }));
    };

    @Override
    public SequentialCommandGroup stop() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    this.stopArm();
                }));
    };

}