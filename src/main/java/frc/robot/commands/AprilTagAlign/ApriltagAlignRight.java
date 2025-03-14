package frc.robot.commands.AprilTagAlign;

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
 * Command to align the robot with the nearest AprilTag using PhotonVision while
 * still allowing translational driving.
 */

public class ApriltagAlignRight extends Command implements VisionIO {
  private final Drive m_drivetrain;
  private final Joystick m_translator;
  private final PIDController m_controller;
  private final PhotonCamera camera1;
  private double m_turnError;
  private double m_turnPower;
  private DoubleSupplier xSupplier;
  private DoubleSupplier ySupplier;

  private Rotation2d rotation;
  private PIDController pidController;
  private double targetAngle;

  public ApriltagAlignRight(Joystick joystick, Drive drivetrain,
      DoubleSupplier x_Supplier,
      DoubleSupplier y_Supplier) {
    m_drivetrain = drivetrain;
    m_translator = joystick;
    xSupplier = x_Supplier;
    ySupplier = y_Supplier;
    m_controller = new PIDController(Constants.VisionConstants.kPTurn, Constants.VisionConstants.kITurn,
        Constants.VisionConstants.kDTurn);
    camera1 = new PhotonCamera(VisionConstants.outtake_Cam);

    this.rotation = drivetrain.getRotation();
    targetAngle = 0;
    // Initialize PID controller (tune these values as needed)
    this.pidController = new PIDController(0.006, 0.0, 0.0);
    pidController.enableContinuousInput(-180, 180);
    pidController.setTolerance(1); // Allowable error in degrees

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
    rotation = m_drivetrain.getRotation();
    // Read in relevant data from the Camera
    boolean targetVisible = false;
    double targetYaw = 0.0;
    double targetRange = 0.0;
    double yawThreshold = 1.0;
    double strafeSpeed = 0.0;

    PhotonPipelineResult results = camera1.getLatestResult();
    if (results.hasTargets()) {
      // At least one AprilTag was seen by the camera
      PhotonTrackedTarget target = results.getBestTarget();
      targetYaw = target.getYaw();
      targetRange = PhotonUtils.calculateDistanceToTargetMeters(
          0.5, // Camera Height
          1.435, // Target Height
          Units.degreesToRadians(-30.0), // Camera Pitch
          Units.degreesToRadians(target.getPitch()));
      // FIND ANGLE FROM ID
      switch(target.getFiducialId()) {
        // BLUE SIDE TARGETS 18 is closet to DS
        case 22:
          targetAngle = -30;
          break;
        case 21:
          targetAngle = -90;
          break;
        case 20:
          targetAngle = -150;
          break;
        case 19:
          targetAngle = 150;
          break;
        case 18:
          targetAngle = 90;
          break;
        case 17:
          targetAngle = 90;
          break;
        // RED SIDE TARGERTS 9 -- 22
        case 9:
          targetAngle = -30;
          break;
        case 10:
          targetAngle = -90;
          break;
        case 11:
          targetAngle = -150;
          break;
        case 6:
          targetAngle = 150;
          break;
        case 7:
          targetAngle = 90;
          break;
        case 8:
          targetAngle = 90;
          break;
        default:
          // code block
          targetAngle = 0;
      }
      pidController.setSetpoint(targetAngle);
      targetVisible = true;
    }

    // OFFSET PRESETS FOR DIFF OBJECTS (THIS IS LEFT)
    double finTargetOffset = Units.inchesToMeters(6.47);
    double cameraToArmOffset = Units.inchesToMeters(6); // add when going right,
    // subtract when going left
    double angleOffset = Units.radiansToDegrees(Math.atan((finTargetOffset +
    cameraToArmOffset) / targetRange)); // use trig to calculate angle
    targetYaw -= angleOffset; // subtract offset to go right add to go left

    // Override the driver's turn command with an automatic one that turns toward
    // the tag.
    double lateralSpeed = -1.0 * targetYaw * Constants.VisionConstants.kPTurn * DriveConstants.kMaxSpeedMetersPerSecond;
    double forward = -1.0 * targetRange * Constants.VisionConstants.kPStrafe * DriveConstants.kMaxSpeedMetersPerSecond;
    // Put debug information to the dashboard
    SmartDashboard.putBoolean("Vision Target Visible", targetVisible);
    SmartDashboard.putNumber("Target Yaw", targetYaw);
    // Command drivetrain motors based on target speeds
    double linearMagnitude = MathUtil.applyDeadband(
        Math.hypot(
            xSupplier.getAsDouble(), ySupplier.getAsDouble()),
        0.02);
    Rotation2d linearDirection = new Rotation2d(
        xSupplier.getAsDouble(), ySupplier.getAsDouble());
    double omega = MathUtil.applyDeadband(0.5, 0.02);

    // Get the current gyro angle
    double currentAngle = rotation.getDegrees();

    // Calculate the rotation speed using the PID controller
    double rotationSpeed = pidController.calculate(currentAngle);

    // Limit the speed to prevent over-rotation
    rotationSpeed = Math.max(-1.0, Math.min(1.0, rotationSpeed));
    // Square values
    linearMagnitude = linearMagnitude * linearMagnitude;
    // Calcaulate new linear velocity
    Translation2d linearVelocity = new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();

    // Convert to robot relative speeds & send command
    m_drivetrain.runVelocity(
        ChassisSpeeds.fromRobotRelativeSpeeds(
            linearVelocity.getX() + lateralSpeed * m_drivetrain.getMaxLinearSpeedMetersPerSec(),
            linearVelocity.getY() + forward * m_drivetrain.getMaxLinearSpeedMetersPerSec(),
            m_drivetrain.getMaxAngularSpeedRadPerSec() * rotationSpeed,
            new Rotation2d()));

    if (targetYaw > 0) {
      m_drivetrain.runVelocity(
          ChassisSpeeds.fromRobotRelativeSpeeds(
              m_drivetrain.getMaxLinearSpeedMetersPerSec() / 10,
              0,
              0,
              new Rotation2d()));

    }

    if (Math.abs(targetYaw) < yawThreshold) {
      lateralSpeed = 0;
    }

  }

  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putString("AprilTagAlign", interrupted ? "Interrupted" : "Completed");
  }

  @Override
  public boolean isFinished() {
    // End command when turn error is within tolerance
    return m_controller.atSetpoint() && pidController.atSetpoint();
  }
}
