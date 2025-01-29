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
import frc.robot.subsystems.arm.ArmIOConstants;

public class ArmIOSIm implements ArmIO{
    private static final double LOOP_PERIOD_SECS = 0.02;
    private static final DCMotor armMotorModel = DCMotor.getNeo550(ArmIOConstants.kArmCANID);

    public static final double armReduction = (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0);

    private final DCMotorSim armSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(armMotorModel, 0.004, armReduction),
        armMotorModel);
    private final EncoderSim encoder = new EncoderSim(new Encoder(DigitalSource(), null));

    
    public void updateInputs(ArmIOInputs inputs) {
        ArmIOSIm.updateInputs(LOOP_PERIOD_SECS); 
        ArmIO.ArmIOInputs.armAppliedVolts = getVoltage();
        ArmIO.ArmIOInputs.armPositionRad = getArmAngleRad();
        ArmIO.ArmIOInputs.armPositionDegrees = getArmAngleDegrees();
        ArmIO.ArmIOInputs.armVelocityMeterPerSec = getArmVelocity();
        ArmIO.ArmIOInputs.armVelocityRadPerSec = getArmVelocity();
    }
    @Override
    public double getVoltage(){
        return armSim.getInputVoltage();
    }

    @Override
    public double getArmAngleRad(){
        return 0;
        //this is not right
    }

    @Override
    public double getArmAngleDegrees(){
        return 0;
        //this is not righth
    }

    @Override
    public double getArmVelocity(){
        return 0;
    }

    public void setArmPosition(double kArmPositionPlaceholder, ArmIOInputs inputs) {}
    @Override
    public SequentialCommandGroup clockwise(){return new SequentialCommandGroup(null);};
    @Override
    public SequentialCommandGroup counterClockwise(){return new SequentialCommandGroup(null);};
    @Override
    public SequentialCommandGroup stop(){return new SequentialCommandGroup(null);};
}
