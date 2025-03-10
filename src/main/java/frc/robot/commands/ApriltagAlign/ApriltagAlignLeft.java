package frc.robot.commands.ApriltagAlign;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.PhotonVision.VisionConstants;
import frc.robot.subsystems.PhotonVision.VisionIO;
import frc.robot.subsystems.PhotonVision.VisionIOPhoton.*;
import frc.robot.subsystems.PhotonVision.VisionIO.*;

import java.util.function.DoubleSupplier;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * Command to align the robot with the nearest AprilTag using PhotonVision while still allowing translational driving.
 */

public class ApriltagAlignLeft extends Command {
  private final Drive m_drivetrain;
  private final Joystick m_translator;
  private final PIDController m_controller;
  private final PhotonCamera camera1;
  private double m_turnError;
  private double m_turnPower;
  private DoubleSupplier xSupplier;
  private DoubleSupplier ySupplier;

  public ApriltagAlignLeft(Joystick joystick, Drive drivetrain,
  DoubleSupplier x_Supplier,
  DoubleSupplier y_Supplier) {
    m_drivetrain = drivetrain;
    m_translator = joystick;
    xSupplier = x_Supplier;
    ySupplier = y_Supplier;
    m_controller = new PIDController(Constants.VisionConstants.kPTurn, Constants.VisionConstants.kITurn, Constants.VisionConstants.kDTurn);
    camera1 = new PhotonCamera(VisionConstants.outtake_Cam);
    addRequirements(m_drivetrain);
  }

  @Override
  public void initialize() {
    m_controller.reset();
    m_turnError = 0;
    m_turnPower = 0;
    SmartDashboard.putString("AprilTagAlign", "Initialized");
  }

  @Override
  public void execute() {

    // Read in relevant data from the Camera
    boolean targetVisible = false;
    double targetYaw = 0.0;
    double targetRange = 0.0;

    PhotonPipelineResult results = camera1.getLatestResult();
    if (results.hasTargets()) {
      // At least one AprilTag was seen by the camera
      PhotonTrackedTarget target = results.getBestTarget();
      targetYaw = target.getYaw();
      targetRange =
                                PhotonUtils.calculateDistanceToTargetMeters(
                                        0.5, // Camera Height
                                        1.435, // Target Height
                                        Units.degreesToRadians(-30.0), //Camera Pitch
                                        Units.degreesToRadians(target.getPitch()));
      targetVisible = true;
    }

    // OFFSET PRESETS FOR DIFF OBJECTS (THIS IS LEFT)
    double finTargetOffset = Units.inchesToMeters(6.47);
    double cameraToArmOffset = Units.inchesToMeters(6); // add when going right, subtract when going left
    double angleOffset = Units.radiansToDegrees(Math.atan((finTargetOffset - cameraToArmOffset) / targetRange)); // use trig to calculate angle
    targetYaw += angleOffset; // subtract offset to go right add to go left

    // Override the driver's turn command with an automatic one that turns toward the tag.
    double turn = -1.0 * targetYaw * Constants.VisionConstants.kPTurn * DriveConstants.kMaxAngularSpeedRadiansPerSecond;
    double forward = -1.0 * targetRange * Constants.VisionConstants.kPStrafe * DriveConstants.kMaxSpeedMetersPerSecond; 
  // Put debug information to the dashboard
  SmartDashboard.putBoolean("Vision Target Visible", targetVisible);
  SmartDashboard.putNumber("Target Yaw", targetYaw);
  SmartDashboard.putNumber("Turn Power", turn);
    // Command drivetrain motors based on target speeds
    double linearMagnitude =
              MathUtil.applyDeadband(
                  Math.hypot(
                      xSupplier.getAsDouble(), ySupplier.getAsDouble()),
                  0.02);
          Rotation2d linearDirection =
              new Rotation2d(
                  xSupplier.getAsDouble(), ySupplier.getAsDouble());
          double omega = MathUtil.applyDeadband(turn*0.5, 0.02);

          // Square values
          linearMagnitude = linearMagnitude * linearMagnitude;
          // Calcaulate new linear velocity
          Translation2d linearVelocity =
              new Pose2d(new Translation2d(), linearDirection)
                  .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                  .getTranslation();

          // Convert to robot relative speeds & send command
          m_drivetrain.runVelocity(
              ChassisSpeeds.fromRobotRelativeSpeeds(
                  linearVelocity.getX() * m_drivetrain.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() + forward * m_drivetrain.getMaxLinearSpeedMetersPerSec(),
                  omega * m_drivetrain.getMaxAngularSpeedRadPerSec(),
                  new Rotation2d()));

    
  }

  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putString("AprilTagAlign", interrupted ? "Interrupted" : "Completed");
  }

  @Override
  public boolean isFinished() {
    // End command when turn error is within tolerance
    return m_controller.atSetpoint();
  }
}
