package frc.robot.subsystems.ApriltagAlign;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
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

  public final PhotonCamera camera1 = new PhotonCamera(VisionConstants.cam1Name);

  private double m_turnPower;

  /**
   * Creates a new AprilTagAlign.
   *
   * @param joystick    The joystick for translational control.
   * @param drivetrain  The drivetrain subsystem.
   */
  public ApriltagAlign(Joystick joystick, Drive drivetrain) {
    m_drivetrain = drivetrain;
    m_translator = joystick;
    m_controller = new PIDController(Constants.VisionConstants.kPTurn, Constants.VisionConstants.kITurn, Constants.VisionConstants.kDTurn);

    // Reset PID controller and set tolerance
    m_controller.reset();
    m_controller.setTolerance(Constants.VisionConstants.aprilTagAlignTolerance);

    // Declare subsystem dependencies
    addRequirements(m_drivetrain);
  }

  @Override
  public void initialize() {
    m_turnPower = 0.0;
    SmartDashboard.putString("AprilTagAlign", "Initialized");
  }

  @Override
  public void execute() {
    var result = getLatestResult(camera1);

    if (result.hasTargets()) {
      PhotonTrackedTarget target = result.getBestTarget();
      double targetYaw = target.getYaw();

      // Calculate turn power using PID controller
      m_turnPower = m_controller.calculate(targetYaw);

      // Apply small stabilization gain
      m_turnPower += Math.copySign(Constants.VisionConstants.kSTurn, targetYaw);

      // Constrain turn power
      m_turnPower = MathUtil.clamp(m_turnPower, -1.0, 1.0);

      // Stop turning if error is within tolerance
      if (Math.abs(targetYaw) < Constants.VisionConstants.aprilTagAlignTolerance) {
        m_turnPower = 0.0;
      }

      // Send debug information to SmartDashboard
      SmartDashboard.putNumber("Target Yaw", targetYaw);
      SmartDashboard.putNumber("Turn Power", m_turnPower);

      // Drive the robot with translational and rotational inputs
      CommandScheduler.getInstance()
          .schedule(
              frc.robot.commands.DriveCommands.joystickDrive(
                  m_drivetrain,
                  () -> MathUtil.applyDeadband(
                      -m_translator.getY() * Constants.OperatorConstants.translationMultiplier,
                      Constants.OperatorConstants.kDriveDeadband),
                  () -> MathUtil.applyDeadband(
                      -m_translator.getX() * Constants.OperatorConstants.translationMultiplier,
                      Constants.OperatorConstants.kDriveDeadband),
                  () -> m_turnPower));

    } else {
      // No target detected: Do nothing and clear SmartDashboard indicators
      m_turnPower = 0.0;
      SmartDashboard.putString("AprilTagAlign", "No Targets - Idle");
    }
  }

  @Override
  public void end(boolean interrupted) {
    CommandScheduler.getInstance()
        .schedule(
            frc.robot.commands.DriveCommands.joystickDrive(
                m_drivetrain,
                () -> 0.0,  // X translation (stopped)
                () -> 0.0,  // Y translation (stopped)
                () -> 0.0  // Rotation (stopped)
            ));
    SmartDashboard.putString("AprilTagAlign", interrupted ? "Interrupted" : "Completed");
  }

  @Override
  public boolean isFinished() {
    return m_controller.atSetpoint();
  }
}
