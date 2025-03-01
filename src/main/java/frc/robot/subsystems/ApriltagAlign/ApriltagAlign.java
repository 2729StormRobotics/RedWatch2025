package frc.robot.subsystems.ApriltagAlign;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
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
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * Command to align the robot with the nearest AprilTag using PhotonVision while still allowing translational driving.
 */

public class ApriltagAlign extends Command implements VisionIO {
  private final Drive m_drivetrain;
  private final Joystick m_translator;
  private final PIDController m_controller;
  private final PhotonCamera camera1;
  private double m_turnError;
  private double m_turnPower;

  public ApriltagAlign(Joystick joystick, Drive drivetrain) {
    m_drivetrain = drivetrain;
    m_translator = joystick;
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
    // Calculate drivetrain commands from Joystick values
    double forward = -m_translator.getY() * DriveConstants.kMaxSpeedMetersPerSecond;
    double strafe = -m_translator.getX() * DriveConstants.kMaxSpeedMetersPerSecond;
    // double turn = -m_translator.getTwist() * DriveConstants.kMaxAngularSpeedRadiansPerSecond;s

    // Read in relevant data from the Camera
    boolean targetVisible = false;
    double targetYaw = 0.0;

    PhotonPipelineResult results = camera1.getLatestResult();
    if (results.hasTargets()) {
      // At least one AprilTag was seen by the camera
      PhotonTrackedTarget target = results.getBestTarget();
      targetYaw = target.getYaw();
      targetVisible = true;
    }

    // Override the driver's turn command with an automatic one that turns toward the tag.
    double turn = -1.0 * targetYaw * Constants.VisionConstants.kPTurn * DriveConstants.kMaxAngularSpeedRadiansPerSecond;

    // Command drivetrain motors based on target speeds
    CommandScheduler.getInstance().schedule(
        frc.robot.commands.DriveCommands.joystickDrive(
            m_drivetrain,
            () -> forward,
            () -> strafe,
            () -> turn
        )
    );

    // Put debug information to the dashboard
    SmartDashboard.putBoolean("Vision Target Visible", targetVisible);
    SmartDashboard.putNumber("Target Yaw", targetYaw);
    SmartDashboard.putNumber("Turn Power", turn);
    SmartDashboard.putNumber("Forward Power", forward);
    SmartDashboard.putNumber("Strafe Power", strafe);
  }

  @Override
  public void end(boolean interrupted) {
    // Restore manual control by scheduling the joystick drive command
    CommandScheduler.getInstance().schedule(
        frc.robot.commands.DriveCommands.joystickDrive(
            m_drivetrain,
            () -> -m_translator.getY() * OperatorConstants.translationMultiplier,
            () -> -m_translator.getX() * OperatorConstants.translationMultiplier,
            () -> m_translator.getTwist() * OperatorConstants.rotationMultiplier
        )
    );
    SmartDashboard.putString("AprilTagAlign", interrupted ? "Interrupted" : "Completed");
  }

  @Override
  public boolean isFinished() {
    // End command when turn error is within tolerance
    return m_controller.atSetpoint();
  }
}
