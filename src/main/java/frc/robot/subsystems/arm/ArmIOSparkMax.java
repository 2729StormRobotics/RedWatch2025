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
import frc.robot.subsystems.arm.ArmConstants;

public class ArmIOSparkMax implements ArmIO {
    public SparkMax armSparkMaxLeft;
    public SparkMax armSparkMaxRight;
    public SparkMaxConfig armConfigRight;
    public SparkMaxConfig armConfigLeft;
    public SparkLimitSwitch metalDetector;
    public AbsoluteEncoder armAbsoluteEncoder;

    public ArmIOSparkMax() {
        // Define motor
        armSparkMaxLeft = new SparkMax(ArmConstants.kArmCANID, MotorType.kBrushless);
        armSparkMaxRight = new SparkMax(ArmConstants.kArmCANID2, MotorType.kBrushless);

        // Define Configs for Hanger Motor

        armConfigRight = new SparkMaxConfig();
        armConfigRight.closedLoop.pid(ArmConstants.kPArm, ArmConstants.kIArm, ArmConstants.kDArm);
        armConfigRight.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        armConfigRight.closedLoop.outputRange(ArmConstants.kArmMinOutputPower, ArmConstants.kArmMaxOutputPower);
        armConfigRight.idleMode(IdleMode.kBrake);

        armConfigRight.absoluteEncoder.velocityConversionFactor(0.10472);

        // burn motor
        armSparkMaxLeft.configure(armConfigRight, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        armSparkMaxRight.configure(armConfigRight, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        metalDetector = armSparkMaxLeft.getForwardLimitSwitch();
        armAbsoluteEncoder = armSparkMaxRight.getAbsoluteEncoder();

        armConfigLeft = new SparkMaxConfig();
        armConfigLeft.follow(armSparkMaxRight, false);
    }

    public void updateInputs(ArmIOInputs inputs) {
        ArmIO.ArmIOInputs.armCLC = armConfigRight.closedLoop;
        ArmIO.ArmIOInputs.armAppliedVolts = (armSparkMaxLeft.getBusVoltage() * armSparkMaxLeft.getAppliedOutput()); // divided
                                                                                                              // by some
                                                                                                              // number?
        ArmIO.ArmIOInputs.armPositionRad = getArmAngleRad();
        ArmIO.ArmIOInputs.armPositionDegrees = getArmAngleDegrees();
        ArmIO.ArmIOInputs.armVelocityRadPerSec = getArmVelocity();
    }

    @Override
    public void pivotClockwise() {

        armSparkMaxRight.set(ArmConstants.kArmMotorSpeed);
    }

    @Override
    public double getVoltage() {
        return armSparkMaxLeft.getBusVoltage();
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

        armSparkMaxRight.set(-ArmConstants.kArmMotorSpeed);
    };

    @Override
    public void setVoltage(double voltage) {
        armSparkMaxRight.setVoltage(voltage);
    }

    public void goAngle(double kAngle) {
        armSparkMaxRight.getClosedLoopController().setReference(kAngle, ControlType.kPosition);
    }

    @Override
    public void stopArm() {
        armSparkMaxRight.stopMotor();
    };

    @Override
    public void setP(double p) {
        ArmConstants.kPArm = p;
        armConfigRight.closedLoop.p(p);
    }

    @Override
    public void setI(double i) {
        ArmConstants.kIArm = i;
        armConfigRight.closedLoop.i(i);
    }

    @Override
    public void setD(double d) {
        ArmConstants.kDArm = d;
        armConfigRight.closedLoop.d(d);
    }

    @Override
    public double getP() {
        return ArmConstants.kPArm;
    }

    @Override
    public double getI() {
        return ArmConstants.kIArm;
    }

    @Override
    public double getD() {
        return ArmConstants.kDArm;
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