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
  }

  @Override
  public void fromLog(LogTable table) {
    motor = table.get("Motor", motor);
    pidController = table.get("PidController", pidController);
    feedforwardController = table.get("FeedforwardController", feedforwardController);
  }

  public ElevatorIOInputsAutoLogged clone() {
    ElevatorIOInputsAutoLogged copy = new ElevatorIOInputsAutoLogged();
    copy.motor = this.motor;
    copy.pidController = this.pidController;
    copy.feedforwardController = this.feedforwardController;
    return copy;
  }
}
