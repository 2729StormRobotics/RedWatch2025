package frc.robot.subsystems.arm;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.arm.ArmIO.ArmIOInputs;
import frc.robot.subsystems.arm.ArmIOConstants;

public class ArmIOSIm {
    private static final double LOOP_PERIOD_SECS = 0.02;
    private static final DCMotor armMotorModel = DCMotor.getNeo550(ArmIOConstants.kArmCANID);

    public static final double armReduction = (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0);

    private final DCMotorSim armSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(armMotorModel, 0.004, armReduction),
        armMotorModel);
    
    public static void updateInputs(ArmIOInputs inputs) {
          ArmIOSIm.updateInputs(inputs);
    }

    public void setArmPosition(double kArmPositionPlaceholder, ArmIOInputs inputs) {}
}
