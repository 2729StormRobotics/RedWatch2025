package frc.robot.subsystems.hanger;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/**
 * Simulation implementation of the HangerIO interface.
 */
public class HangerIOSim implements HangerIO {

  private static final DCMotor hangerMotorModel = DCMotor.getNeo550(HangerConstants.kWinchMotorCanId); // Change to
                                                                                                       // whatever CAN
                                                                                                       // ID it is
  public static boolean isClosed = false;
  private final DCMotorSim hangerSim = new DCMotorSim(
      LinearSystemId.createDCMotorSystem(hangerMotorModel, 0.004, HangerConstants.hangerReduction),
      hangerMotorModel);

  /**
   * Sets the voltage of the simulated hanger motor.
   *
   * @param volts The voltage to set.
   */
  @Override
  public void setHangerVoltage(double volts) {
    hangerSim.setInputVoltage(volts);
  }

  /**
   * Gets the voltage of the simulated hanger motor.
   *
   * @return The voltage.
   */
  @Override
  public double getHangerVoltage() {
    return hangerSim.getInputVoltage();
  }

  /**
   * Gets the angle of the simulated hanger.
   *
   * @return The angle (always 0 in simulation).
   */
  @Override
  public double getHangerAngle() {
    // NEEDS TO BE TESTED
    return 0;
  }

  /**
   * Pulls the simulated hanger mechanism.
   */
  @Override
  public void pull() {
    hangerSim.setInputVoltage(HangerConstants.motorSpeed);
    isClosed = true;
  }

  /**
   * Releases the simulated hanger mechanism.
   */
  @Override
  public void release() {
    hangerSim.setInputVoltage(-HangerConstants.motorSpeed);
    isClosed = false;
  }

  /**
   * Stops the simulated hanger motor.
   */
  @Override
  public void stop() {
    hangerSim.setInputVoltage(0);
  }

  /**
   * Creates a command group to retract the simulated hanger.
   *
   * @return The retract command group.
   */
  @Override
  public SequentialCommandGroup retract() {
    return new SequentialCommandGroup(
        new InstantCommand(() -> {
          this.pull();
        }),
        new WaitCommand(0.5),
        new InstantCommand(() -> {
          this.stop();
        }));
  }

  /**
   * Creates a command group to extend the simulated hanger.
   *
   * @return The extend command group.
   */
  @Override
  public SequentialCommandGroup extend() {
    return new SequentialCommandGroup(
        new InstantCommand(() -> {
          this.release();
        }),
        new WaitCommand(0.5),
        new InstantCommand(() -> {
          this.stop();
        }));
  }
}