package frc.robot.subsystems.elevator;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ElevatorIOInputsAutoLogged extends ElevatorIO.ElevatorIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("Motor", motor);
    table.put("PidController", pidController);
    table.put("FeedforwardController", feedforwardController);
    table.put("ElevatorPot", elevatorPot);
    table.put("ElevatorAppliedVolts", elevatorAppliedVolts);
    table.put("ElevatorCurrentAmps", elevatorCurrentAmps);
    table.put("ElevatorPositionRad", elevatorPositionRad);
    table.put("KWheelDiameterMeters", kWheelDiameterMeters);
    table.put("ElevatorPositionMeters", elevatorPositionMeters);
    table.put("ElevatorVelocityMeterPerSec", elevatorVelocityMeterPerSec);
    table.put("ElevatorVelocityRadPerSec", elevatorVelocityRadPerSec);
  }

  @Override
  public void fromLog(LogTable table) {
    motor = table.get("Motor", motor);
    pidController = table.get("PidController", pidController);
    feedforwardController = table.get("FeedforwardController", feedforwardController);
    elevatorPot = table.get("ElevatorPot", elevatorPot);
    elevatorAppliedVolts = table.get("ElevatorAppliedVolts", elevatorAppliedVolts);
    elevatorCurrentAmps = table.get("ElevatorCurrentAmps", elevatorCurrentAmps);
    elevatorPositionRad = table.get("ElevatorPositionRad", elevatorPositionRad);
    kWheelDiameterMeters = table.get("KWheelDiameterMeters", kWheelDiameterMeters);
    elevatorPositionMeters = table.get("ElevatorPositionMeters", elevatorPositionMeters);
    elevatorVelocityMeterPerSec = table.get("ElevatorVelocityMeterPerSec", elevatorVelocityMeterPerSec);
    elevatorVelocityRadPerSec = table.get("ElevatorVelocityRadPerSec", elevatorVelocityRadPerSec);
  }

  public ElevatorIOInputsAutoLogged clone() {
    ElevatorIOInputsAutoLogged copy = new ElevatorIOInputsAutoLogged();
    copy.motor = this.motor;
    copy.pidController = this.pidController;
    copy.feedforwardController = this.feedforwardController;
    copy.elevatorPot = this.elevatorPot;
    copy.elevatorAppliedVolts = this.elevatorAppliedVolts;
    copy.elevatorCurrentAmps = this.elevatorCurrentAmps.clone();
    copy.elevatorPositionRad = this.elevatorPositionRad;
    copy.kWheelDiameterMeters = this.kWheelDiameterMeters;
    copy.elevatorPositionMeters = this.elevatorPositionMeters;
    copy.elevatorVelocityMeterPerSec = this.elevatorVelocityMeterPerSec;
    copy.elevatorVelocityRadPerSec = this.elevatorVelocityRadPerSec;
    return copy;
  }
}
