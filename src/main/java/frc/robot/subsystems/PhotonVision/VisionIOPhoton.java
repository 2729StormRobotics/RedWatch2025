  // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//FIX CAM NAMES *******************************
// ******

package frc.robot.subsystems.PhotonVision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.PhotonVision.VisionIO.VisionIOInputs;

import static frc.robot.subsystems.PhotonVision.VisionConstants.AMBIGUITY_THRESHOLD;
import static frc.robot.subsystems.PhotonVision.VisionConstants.cam1Name;
import static frc.robot.subsystems.PhotonVision.VisionConstants.cam1RobotToCam;
import static frc.robot.subsystems.PhotonVision.VisionConstants.cam2Name;
import static frc.robot.subsystems.PhotonVision.VisionConstants.cam2RobotToCam;
import static frc.robot.subsystems.PhotonVision.VisionConstants.cam3Name;
import static frc.robot.subsystems.PhotonVision.VisionConstants.cam3RobotToCam;
import static frc.robot.subsystems.PhotonVision.VisionConstants.kTagLayout;


import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class VisionIOPhoton implements VisionIO {
  
  public final PhotonCamera camera1;
  public final PhotonPoseEstimator camera1Estimator;

  public final PhotonCamera camera2;
  public final PhotonPoseEstimator camera2Estimator;

  public final PhotonCamera camera3;
  public final PhotonPoseEstimator camera3Estimator;

  private Pose2d lastEstimate = new Pose2d(); 

   LoggedDashboardBoolean killSideCams = new LoggedDashboardBoolean("Vision/KillSideCams", false); // **COMMENTED OUT FUNCTION CUZ ITS WEIRD**


  // Initialzes camera with a name and creates a PhotonPoseEstimator to process vision data
  public VisionIOPhoton() {
    PortForwarder.add(5800, "photonvision.local", 5800);

    // Camera 1
    camera1 = new PhotonCamera(cam1Name);
    camera1Estimator = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cam1RobotToCam);
    camera1Estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    // Camera 2
    camera2 = new PhotonCamera(cam2Name);
    camera2Estimator = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cam2RobotToCam);
    camera2Estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    // Camera 3
    camera3 = new PhotonCamera(cam3Name);
    camera3Estimator = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cam3RobotToCam);
    camera3Estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

  }

  // Defines PhotonPoseEstimator which sets current estimate
  private PhotonPoseEstimator[] getAprilTagEstimators(Pose2d currentEstimate) {
    camera1Estimator.setReferencePose(currentEstimate);
    camera2Estimator.setReferencePose(currentEstimate);
    camera3Estimator.setReferencePose(currentEstimate);

    return new PhotonPoseEstimator[] { camera1Estimator, camera2Estimator, camera3Estimator };
  }

  // Updates inputs for vision processing
  @Override
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
      inputs.camera1Targets = cameraTargets[0];

      Pose3d[] tags = getTargetsPositions(results);
      Logger.recordOutput("Vision/Targets3D", tags); 
      Logger.recordOutput("Vision/Targets", Pose3dToPose2d(tags)); 
      Logger.recordOutput("Vision/TagCounts", tagCounts(results)); 
    } 
    else {
      inputs.timestamp = inputs.timestamp;
      inputs.hasEstimate = false;
    }

    Logger.recordOutput("Vision/cam1/Connected", camera1.isConnected());
    Logger.recordOutput("Vision/cam2/Connected", camera2.isConnected());
    Logger.recordOutput("Vision/cam3/Connected", camera3.isConnected());
  }

  private void printStuff(String name, PhotonPipelineResult result) {
    Logger.recordOutput("Vision/" + name + "/results", result.getTargets().size()); 

    PhotonTrackedTarget target = result.getBestTarget();
    if (target != null) {
        Logger.recordOutput("Vision/" + name + "/PoseAmbiguity", result.getBestTarget().getPoseAmbiguity());
        Logger.recordOutput("Vision/" + name + "/Yaw", result.getBestTarget().getYaw());
    }
  }
 
  public PhotonPipelineResult[] getAprilTagResults() {
    PhotonPipelineResult cam1_result = getLatestResult(camera1);
    PhotonPipelineResult cam2_result = getLatestResult(camera2);
    PhotonPipelineResult cam3_result = getLatestResult(camera3);
    printStuff("cam1", cam1_result);
    printStuff("cam2", cam2_result);
    printStuff("cam3", cam3_result);

    return new PhotonPipelineResult[] { cam1_result, cam2_result, cam3_result };
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


  /**
   * The standard deviations of the estimated pose from {@link #getEstimatedGlobalPose()}, for use
   * with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}.
   * This should only be used when there are targets visible.
   *
   * @param estimatedPose The estimated pose to guess standard deviations for.
   */
}