package frc.robot.subsystems.PhotonVision;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class VisionIOInputsAutoLogged extends VisionIO.VisionIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("Estimate", estimate);
    table.put("Timestamp", timestamp);
    table.put("TimestampArray", timestampArray);
    table.put("Camera1Targets", camera1Targets);
    table.put("Camera2Targets", camera2Targets);
    table.put("Camera3Targets", camera3Targets);
    table.put("HasEstimate", hasEstimate);
    table.put("Results", results);
  }

  @Override
  public void fromLog(LogTable table) {
    estimate = table.get("Estimate", estimate);
    timestamp = table.get("Timestamp", timestamp);
    timestampArray = table.get("TimestampArray", timestampArray);
    camera1Targets = table.get("Camera1Targets", camera1Targets);
    camera2Targets = table.get("Camera2Targets", camera2Targets);
    camera3Targets = table.get("Camera3Targets", camera3Targets);
    hasEstimate = table.get("HasEstimate", hasEstimate);
    results = table.get("Results", results);
  }

  public VisionIOInputsAutoLogged clone() {
    VisionIOInputsAutoLogged copy = new VisionIOInputsAutoLogged();
    copy.estimate = this.estimate.clone();
    copy.timestamp = this.timestamp;
    copy.timestampArray = this.timestampArray.clone();
    copy.camera1Targets = this.camera1Targets.clone();
    copy.camera2Targets = this.camera2Targets.clone();
    copy.camera3Targets = this.camera3Targets.clone();
    copy.hasEstimate = this.hasEstimate;
    copy.results = this.results.clone();
    return copy;
  }
}
