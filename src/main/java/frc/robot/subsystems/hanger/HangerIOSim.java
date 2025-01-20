package frc.robot.subsystems.hanger;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class HangerIOSim implements HangerIO {
  private static final double LOOP_PERIOD_SECS = 0.02;

  private static final DCMotor hangerMotorModel = DCMotor.getNeo550(HangerConstants.kWinchMotorCanId); // Change to
                                                                                                       // whatever CAN
                                                                                                       // ID it is
  public static boolean isClosed = false;
  private final DCMotorSim hangerSim = new DCMotorSim(
      LinearSystemId.createDCMotorSystem(hangerMotorModel, 0.004, HangerConstants.hangerReduction),
      hangerMotorModel);

  @Override
  public void setHangerVoltage(double volts) {
    hangerSim.setInputVoltage(volts);
  }

  @Override
  public double getHangerVoltage() {
    return hangerSim.getInputVoltage();
  }

  @Override
  public double getHangerAngle() {
    // NEEDS TO BE TESTED
    return 0;
  }

  @Override
  public void pull() {
    hangerSim.setInputVoltage(HangerConstants.motorSpeedOpenHanger);
    isClosed = true;
  }

  @Override
  public void release() {
    hangerSim.setInputVoltage(-HangerConstants.motorSpeedOpenHanger);
    isClosed = false;
  }

  @Override
  public void stop() {
    hangerSim.setInputVoltage(0);
  }

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
