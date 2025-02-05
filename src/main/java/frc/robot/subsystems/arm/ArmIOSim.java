package frc.robot.subsystems.arm;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.ArmIO.ArmIOInputs;
import frc.robot.subsystems.arm.ArmConstants;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class ArmIOSim implements ArmIO {
    private static final double LOOP_PERIOD_SECS = 0.02;
    private static final DCMotor armMotorModel = DCMotor.getNEO(ArmConstants.kArmCANID);

    public static final double armReduction = (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0);

    private final ProfiledPIDController m_controller;
    private final DCMotorSim armSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(armMotorModel, 0.004, armReduction),
            armMotorModel);
    private SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(0, 0);

    // private final EncoderSim encoder = new EncoderSim(new
    // Encoder(DigitalSource(), null));
    private final Encoder m_encoder;
    private final EncoderSim m_encoderSim;
    private SingleJointedArmSim sim = new SingleJointedArmSim(
            armMotorModel,
            ArmConstants.ArmSimConstants.kArmReduction,
            SingleJointedArmSim.estimateMOI(ArmConstants.ArmSimConstants.kArmLength,
                    ArmConstants.ArmSimConstants.kArmMass),
            ArmConstants.ArmSimConstants.kArmLength,
            ArmConstants.ArmSimConstants.kMinAngleRads,
            ArmConstants.ArmSimConstants.kMaxAngleRads,
            true, // change this to true later
            0.1);

    public ArmIOSim() {
        m_encoder = new Encoder(ArmConstants.ArmSimConstants.kEncoderAChannel,
                ArmConstants.ArmSimConstants.kEncoderBChannel);
        m_encoderSim = new EncoderSim(m_encoder);
        m_encoderSim.setDistancePerPulse(ArmConstants.ArmSimConstants.kArmEncoderDistPerPulse);
        m_controller = new ProfiledPIDController(ArmConstants.ArmSimConstants.kArmSimPID[0],
                ArmConstants.ArmSimConstants.kArmSimPID[1], ArmConstants.ArmSimConstants.kArmSimPID[2],
                new TrapezoidProfile.Constraints(2.45, 2.45));

        m_controller.setTolerance(0.1, 0.05);
    }

    public void updateInputs(ArmIOInputs inputs) {
        armSim.update(LOOP_PERIOD_SECS);
        ArmIO.ArmIOInputs.armAppliedVolts = getVoltage();
        ArmIO.ArmIOInputs.armPositionRad = getArmAngleRad();
        ArmIO.ArmIOInputs.armPositionDegrees = getArmAngleDegrees();
        ArmIO.ArmIOInputs.armVelocityRadPerSec = getArmVelocity();
    }

    @Override
    public double getVoltage() {
        return armSim.getInputVoltage();
    }

    @Override
    public double getArmAngleRad() {
        return sim.getAngleRads();
        // this is not right
    }

    @Override
    public double getArmAngleDegrees() {
        return sim.getAngleRads() * (180 / Math.PI);
    }

    @Override
    public double getArmVelocity() {
        return sim.getVelocityRadPerSec();
    }

    public void setArmPosition(double kArmPositionPlaceholder) {
        m_controller.setGoal(kArmPositionPlaceholder);
        // With the setpoint value we run PID control like normal
        double pidOutput = m_controller.calculate(getArmAngleDegrees());
        double feedforwardOutput = m_feedforward.calculate(m_controller.getSetpoint().velocity);

        sim.setInputVoltage(feedforwardOutput + pidOutput);
    }

    @Override
    public SequentialCommandGroup clockwise() {
        return new SequentialCommandGroup(null);
    };

    @Override
    public SequentialCommandGroup counterClockwise() {
        return new SequentialCommandGroup(null);
    };

    @Override
    public SequentialCommandGroup stop() {
        return new SequentialCommandGroup(null);
    };
}