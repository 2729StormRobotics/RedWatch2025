package frc.robot.subsystems.drive;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class GyroIOInputsAutoLogged extends GyroIO.GyroIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("Connected", connected);
    table.put("YawPosition", yawPosition);
    table.put("RollPosition", rollPosition);
    table.put("PitchPosition", pitchPosition);
    table.put("OdometryYawTimestamps", odometryYawTimestamps);
    table.put("OdometryYawPositions", odometryYawPositions);
    table.put("YawVelocityRadPerSec", yawVelocityRadPerSec);
  }

  @Override
  public void fromLog(LogTable table) {
    connected = table.get("Connected", connected);
    yawPosition = table.get("YawPosition", yawPosition);
    rollPosition = table.get("RollPosition", rollPosition);
    pitchPosition = table.get("PitchPosition", pitchPosition);
    odometryYawTimestamps = table.get("OdometryYawTimestamps", odometryYawTimestamps);
    odometryYawPositions = table.get("OdometryYawPositions", odometryYawPositions);
    yawVelocityRadPerSec = table.get("YawVelocityRadPerSec", yawVelocityRadPerSec);
  }

  public GyroIOInputsAutoLogged clone() {
    GyroIOInputsAutoLogged copy = new GyroIOInputsAutoLogged();
    copy.connected = this.connected;
    copy.yawPosition = this.yawPosition;
    copy.rollPosition = this.rollPosition;
    copy.pitchPosition = this.pitchPosition;
    copy.odometryYawTimestamps = this.odometryYawTimestamps.clone();
    copy.odometryYawPositions = this.odometryYawPositions.clone();
    copy.yawVelocityRadPerSec = this.yawVelocityRadPerSec;
    return copy;
  }
}
