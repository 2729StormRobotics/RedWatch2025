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
public class ElevatorIOSIM implements ElevatorIO {
    private static final double LOOP_PERIOD_SECS = 0.02;
  
    private static final DCMotor elevatorMotorModel = DCMotor.getNeoVortex(1);
  
    public static final double elevatorReduction = (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0);
    private final DCMotorSim elevatorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(elevatorMotorModel, 0.025, elevatorReduction), elevatorMotorModel);

    private final Rotation2d turnAbsoluteInitPosition = new Rotation2d(Math.random() * 2.0 * Math.PI);
    public double elevatorAppliedVolts = 0.0;
    public double[] elevatorCurrentAmps;
  
    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
      elevatorSim.update(LOOP_PERIOD_SECS);

      inputs.elevatorAppliedVolts = elevatorAppliedVolts;
      inputs.elevatorCurrentAmps = new double[] {Math.abs(elevatorSim.getCurrentDrawAmps())};
    }
  
    @Override
    public void setElevatorVoltage(double volts) {
      elevatorAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
      elevatorSim.setInputVoltage(elevatorAppliedVolts);
    }
  
    @Override
    public double getElevatorVoltage() {
      return elevatorAppliedVolts;
    }
  
    
}