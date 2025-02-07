package frc.robot.subsystems.gripper;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class GripperIOInputsAutoLogged extends GripperIO.GripperIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("GripperAppliedVolts", gripperAppliedVolts);
    table.put("GripperPositionRad", gripperPositionRad);
    table.put("GripperPositionDegrees", gripperPositionDegrees);
    table.put("GripperVelocityRadPerSec", gripperVelocityRadPerSec);
  }

  @Override
  public void fromLog(LogTable table) {
    gripperAppliedVolts = table.get("GripperAppliedVolts", gripperAppliedVolts);
    gripperPositionRad = table.get("GripperPositionRad", gripperPositionRad);
    gripperPositionDegrees = table.get("GripperPositionDegrees", gripperPositionDegrees);
    gripperVelocityRadPerSec = table.get("GripperVelocityRadPerSec", gripperVelocityRadPerSec);
  }

  public GripperIOInputsAutoLogged clone() {
    GripperIOInputsAutoLogged copy = new GripperIOInputsAutoLogged();
    copy.gripperAppliedVolts = this.gripperAppliedVolts;
    copy.gripperPositionRad = this.gripperPositionRad;
    copy.gripperPositionDegrees = this.gripperPositionDegrees;
    copy.gripperVelocityRadPerSec = this.gripperVelocityRadPerSec;
    return copy;
  }
}
