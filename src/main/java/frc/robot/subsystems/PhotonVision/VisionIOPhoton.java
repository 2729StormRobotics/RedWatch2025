  // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//FIX CAM NAMES *******************************
// ******

package frc.robot.subsystems.PhotonVision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.PhotonVision.VisionIO.VisionIOInputs;

import static frc.robot.subsystems.PhotonVision.VisionConstants.AMBIGUITY_THRESHOLD;
import static frc.robot.subsystems.PhotonVision.VisionConstants.camName;
import static frc.robot.subsystems.PhotonVision.VisionConstants.camRobotToCam;
import static frc.robot.subsystems.PhotonVision.VisionConstants.kTagLayout;


import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class VisionIOPhoton implements VisionIO {
  
  private final PhotonCamera camera;
  private final PhotonPoseEstimator cameraEstimator;

  private Pose2d lastEstimate = new Pose2d(); 

  // Initialzes camera with a name and creates a PhotonPoseEstimator to process vision data
  public VisionIOPhoton() {
    PortForwarder.add(5800, "photonvision.local", 5800);

    camera = new PhotonCamera(camName);
    cameraEstimator = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camRobotToCam);
    cameraEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  // Defines PhotonPoseEstimator which 
  private PhotonPoseEstimator[] getAprilTagEstimators(Pose2d currentEstimate) {
    cameraEstimator.setReferencePose(currentEstimate);
    return new PhotonPoseEstimator[] { cameraEstimator };
  }

  // Updates inputs for vision processing
  public void updateInputs(VisionIOInputs inputs, Pose2d currentEstimate) {
    lastEstimate = currentEstimate;
    
    PhotonPipelineResult[] results = getAprilTagResults();
    PhotonPoseEstimator[] photonEstimators = getAprilTagEstimators(currentEstimate);

    inputs.estimate = new Pose2d[] { new Pose2d() };

    // add code to check if the closest target is in front or back **CHANGE**
    inputs.timestamp = estimateLatestTimestamp(results);

    // Checks for a valid estimate from photonvision
    if (hasEstimate(results)) {
      // inputs.results = results;
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

  Logger.recordOutput("Vision/cam/Connected", camera.isConnected()); 
  }

  private void printStuff(String name, PhotonPipelineResult result) {
    Logger.recordOutput("Vision/" + name + "/results", result.getTargets().size()); 

    PhotonTrackedTarget target = result.getBestTarget();
    if (target != null) {
        Logger.recordOutput("Vision/" + name + "/PoseAmbiguity", result.getBestTarget().getPoseAmbiguity());
        Logger.recordOutput("Vision/" + name + "/Yaw", result.getBestTarget().getYaw());
    }
  }
 
  private PhotonPipelineResult[] getAprilTagResults() {
    PhotonPipelineResult cam_result = getLatestResult(camera);
    printStuff("cam", cam_result);
    return new PhotonPipelineResult[] { cam_result };
  }


  // Checks if a result is valid (checks if has targets and pose amiguity)
  @Override
  public boolean goodResult(PhotonPipelineResult result) {
    return result.hasTargets() && result.getBestTarget().getPoseAmbiguity() < AMBIGUITY_THRESHOLD
        /*
         * && kTagLayout.
         * getTagPose(
         * result.
         * getBestTarget().
         * getFiducialId())
         * .get().toPose2d(
         * ).getTranslation
         * ()
         * .getDistance(
         * lastEstimate.
         * getTranslation()
         * ) < MAX_DISTANCE
         */;
  }
}