package frc.robot.subsystems.Vision_V2;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import frc.robot.subsystems.vision.VisionIO;

public class VisionIOInputsAutoLogged extends VisionIO.VisionIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("PositionEstimates", positionEstimates);
    table.put("RotationEstimates", rotationEstimates);
    table.put("Timestamp", timestamp);
    table.put("TimestampArray", timestampArray);
    table.put("CameraTargets", cameraTargets);
    table.put("HasEstimate", hasEstimate);
    table.put("Results", results);
  }

  @Override
  public void fromLog(LogTable table) {
    positionEstimates = table.get("PositionEstimates", positionEstimates);
    rotationEstimates = table.get("RotationEstimates", rotationEstimates);
    timestamp = table.get("Timestamp", timestamp);
    timestampArray = table.get("TimestampArray", timestampArray);
    cameraTargets = table.get("CameraTargets", cameraTargets);
    hasEstimate = table.get("HasEstimate", hasEstimate);
    results = table.get("Results", results);
  }

  public VisionIOInputsAutoLogged clone() {
    VisionIOInputsAutoLogged copy = new VisionIOInputsAutoLogged();
    copy.positionEstimates = this.positionEstimates.clone();
    copy.rotationEstimates = this.rotationEstimates.clone();
    copy.timestamp = this.timestamp;
    copy.timestampArray = this.timestampArray.clone();
    copy.cameraTargets = this.cameraTargets.clone();
    copy.hasEstimate = this.hasEstimate;
    copy.results = this.results.clone();
    return copy;
  }
}
