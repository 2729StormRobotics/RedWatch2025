package frc.robot.subsystems.PhotonVision;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class VisionIOAutoLogged extends VisionIO implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
  }

  @Override
  public void fromLog(LogTable table) {
  }

  public VisionIOAutoLogged clone() {
    VisionIOAutoLogged copy = new VisionIOAutoLogged();
    return copy;
  }
}
