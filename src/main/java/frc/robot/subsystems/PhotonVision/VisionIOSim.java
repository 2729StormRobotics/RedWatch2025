// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.PhotonVision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.subsystems.PhotonVision.VisionConstants.AMBIGUITY_THRESHOLD;
import static frc.robot.subsystems.PhotonVision.VisionConstants.MAX_DISTANCE;
import static frc.robot.subsystems.PhotonVision.VisionConstants.camName;
import static frc.robot.subsystems.PhotonVision.VisionConstants.camRobotToCam;
import static frc.robot.subsystems.PhotonVision.VisionConstants.getSimVersion;
import static frc.robot.subsystems.PhotonVision.VisionConstants.kMultiTagStdDevs;
import static frc.robot.subsystems.PhotonVision.VisionConstants.kSingleTagStdDevs;
import static frc.robot.subsystems.PhotonVision.VisionConstants.kTagLayout;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class VisionIOSim implements VisionIO {
  private final PhotonCamera cam;
  private final PhotonPoseEstimator camEstimator;
  private PhotonCameraSim camSim;
  private VisionSystemSim visionSim;
  private Pose2d lastEstimate = new Pose2d();

  public VisionIOSim() {
    cam = new PhotonCamera(camName);

    // GOT RID OF CAM AGAIN **CHANGE**
    camEstimator = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, getSimVersion(camRobotToCam));
    camEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    // Create the vision system simulation which handles cams and targets on the field
    visionSim = new VisionSystemSim("main");

    // Add all the AprilTags inside the tag layout as visible targets to this simulated field
    visionSim.addAprilTags(kTagLayout);

    // Create simulated cam properties. These can be set to mimic actual cam
    var camProp = new SimCameraProperties();
    camProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
    camProp.setCalibError(0.35, 0.10);
    camProp.setFPS(15);
    camProp.setAvgLatencyMs(50);
    camProp.setLatencyStdDevMs(15);

    // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible targets.
    camSim = new PhotonCameraSim(cam, camProp);

    // Add the simulated cam1 to view the targets on this simulated field.
    visionSim.addCamera(camSim, getSimVersion(camRobotToCam));

    camSim.enableDrawWireframe(true);
  }

  // Updates robot position and the vision simulation with the current estimate
  @Override
  public void updateInputs(VisionIOInputs inputs, Pose2d currentEstimate) {
    lastEstimate = currentEstimate;
    visionSim.update(currentEstimate);
    camEstimator.setReferencePose(currentEstimate);

    PhotonPipelineResult front_result = getLatestResult(cam);
    
    PhotonPipelineResult[] results = { front_result };
    PhotonPoseEstimator[] photonEstimators = { camEstimator };

    inputs.estimate = new Pose2d[] { new Pose2d() };

    // Add code to check if the closest target is in front or back **CHANGE**
    inputs.timestamp = estimateLatestTimestamp(results);

    // Processes results and updates the inputs if valid estimates are available
    if (hasEstimate(results)) {
      inputs.estimate = getEstimatesArray(results, photonEstimators);
      inputs.hasEstimate = true;
      
      int[][] cameraTargets = getCameraTargets(results);
      inputs.cameraTargets = cameraTargets[0];

      
      Pose3d[] tags = getTargetsPositions(results);
      Logger.recordOutput("Vision/Targets3D", tags);
      Logger.recordOutput("Vision/Targets", Pose3dToPose2d(tags));
      Logger.recordOutput("Vision/TagCounts", tagCounts(results));
    } 
    else {
      inputs.timestamp = inputs.timestamp;
      inputs.hasEstimate = false;
    }

  Logger.recordOutput("Vision/OrangeConnected", cam.isConnected());
  }

  // A Field2d for visualizing our robot and objects on the field
  public Field2d getSimDebugField() {
    return visionSim.getDebugField();
  }

  // Checks if the results are valid (Ambiguity threshold and has targets)
  @Override
  public boolean goodResult(PhotonPipelineResult result) {
    return result.hasTargets() && result.getBestTarget().getPoseAmbiguity() < AMBIGUITY_THRESHOLD
            && kTagLayout.getTagPose(result.getBestTarget().getFiducialId()).get().toPose2d().getTranslation()
                    .getDistance(lastEstimate.getTranslation()) < MAX_DISTANCE;
  }
  
}