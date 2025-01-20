package frc.robot.subsystems.hanger;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class HangerIOInputsAutoLogged extends HangerIO.HangerIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("HangerAngle", HangerAngle);
    table.put("HangerCurrentAmps", HangerCurrentAmps);
  }

  @Override
  public void fromLog(LogTable table) {
    HangerAngle = table.get("HangerAngle", HangerAngle);
    HangerCurrentAmps = table.get("HangerCurrentAmps", HangerCurrentAmps);
  }

  public HangerIOInputsAutoLogged clone() {
    HangerIOInputsAutoLogged copy = new HangerIOInputsAutoLogged();
    copy.HangerAngle = this.HangerAngle;
    copy.HangerCurrentAmps = this.HangerCurrentAmps;
    return copy;
  }
}
