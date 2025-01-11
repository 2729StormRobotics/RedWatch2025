package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * Physics sim implementation of elevator IO.
 *
 * 
 */
public class ElevatorIOSim implements ElevatorIO {
    private static final double LOOP_PERIOD_SECS = 0.02;
  
  
    private static final DCMotor elevatorMotorModel = DCMotor.getNeoVortex(1);
  
    public static final double driveReduction = (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0);
    private final DCMotorSim elevatorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(elevatorMotorModel, 0.025, driveReduction), elevatorMotorModel);

    private double driveAppliedVolts = 0.0;
  
    @Override
    public void updateInputs(ElevatorIOSim inputs) {
      elevatorSim.update(LOOP_PERIOD_SECS);

      inputs.driveAppliedVolts = driveAppliedVolts;
      inputs.driveCurrentAmps = new double[] {Math.abs(elevatorSim.getCurrentDrawAmps())};

    }
  
    @Override
    public void setElevatorVoltage(double volts) {
      driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
      elevatorSim.setInputVoltage(driveAppliedVolts);
    }
  
    @Override
    public double getElevatorVoltage() {
      return driveAppliedVolts;
    }
  
    
}