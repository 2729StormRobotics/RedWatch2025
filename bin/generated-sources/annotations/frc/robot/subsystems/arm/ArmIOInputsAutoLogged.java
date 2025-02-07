package frc.robot.subsystems.arm;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ArmIOInputsAutoLogged extends ArmIO.ArmIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("ArmAppliedVolts", armAppliedVolts);
    table.put("ArmPositionRad", armPositionRad);
    table.put("ArmPositionDegrees", armPositionDegrees);
    table.put("ArmVelocityRadPerSec", armVelocityRadPerSec);
  }

  @Override
  public void fromLog(LogTable table) {
    armAppliedVolts = table.get("ArmAppliedVolts", armAppliedVolts);
    armPositionRad = table.get("ArmPositionRad", armPositionRad);
    armPositionDegrees = table.get("ArmPositionDegrees", armPositionDegrees);
    armVelocityRadPerSec = table.get("ArmVelocityRadPerSec", armVelocityRadPerSec);
  }

  public ArmIOInputsAutoLogged clone() {
    ArmIOInputsAutoLogged copy = new ArmIOInputsAutoLogged();
    copy.armAppliedVolts = this.armAppliedVolts;
    copy.armPositionRad = this.armPositionRad;
    copy.armPositionDegrees = this.armPositionDegrees;
    copy.armVelocityRadPerSec = this.armVelocityRadPerSec;
    return copy;
  }
}
