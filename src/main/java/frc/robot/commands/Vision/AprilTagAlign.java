package frc.robot.commands.Vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.Vision;

/*
 * Makes the robot automatically align to nearest apriltag (while still being able to drive translationally)
 */
public class AprilTagAlign extends Command {
  private final Vision m_vision; 
  private final Drive m_drivetrain;
  private final Joystick m_translator;
  private final PIDController m_controller;
  private double m_turnError = 0.0;
  private double m_turnPower;

  /** Creates a new AprilTagAlign. */
  public AprilTagAlign(Joystick joystick, Drive drivetrain) {
    m_vision = Vision.getInstance();
    m_drivetrain = drivetrain;
    m_translator = joystick;
    m_controller = new PIDController(Constants.VisionConstants.kPTurn, Constants.VisionConstants.kITurn, Constants.VisionConstants.kDTurn);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Reset PID controller and initialize values
    m_controller.reset();
    m_turnError = LimelightHelpers.getTX("limelight");
    m_turnPower = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    while (Math.abs(m_turnError) > Constants.VisionConstants.aprilTagAlignTolerance) {
      m_turnError = LimelightHelpers.getTX("limelight"); // Horizontal angle away from target
      m_turnPower = m_controller.calculate(m_turnError); // Calculate PID output

      // Constrain turnPower to a safe range
      m_turnPower = MathUtil.clamp(m_turnPower, -1.0, 1.0);

      // Stop turning if error is within tolerance
      // if (Math.abs(m_turnError) < Constants.VisionConstants.aprilTagAlignTolerance) {
      //   m_turnPower = 0.0;
      // }

      // Send debug info to SmartDashboard
      SmartDashboard.putNumber("turnError", m_turnError);
      SmartDashboard.putNumber("turnPower", m_turnPower);

      // Drive the robot
      CommandScheduler.getInstance()
          .schedule(
              DriveCommands.joystickDrive(
                  m_drivetrain,
                  () -> MathUtil.applyDeadband(
                      -m_translator.getY() * OperatorConstants.translationMultiplier,
                      OperatorConstants.kDriveDeadband),
                  () -> MathUtil.applyDeadband(
                      -m_translator.getX() * OperatorConstants.translationMultiplier,
                      OperatorConstants.kDriveDeadband),
                  () -> m_turnPower));
    }

    // m_turnPower = 0.0;

    // // Drive the robot
    // CommandScheduler.getInstance()
    // .schedule(
    //     DriveCommands.joystickDrive(
    //         m_drivetrain,
    //         () -> MathUtil.applyDeadband(
    //             -m_translator.getY() * OperatorConstants.translationMultiplier,
    //             OperatorConstants.kDriveDeadband),
    //         () -> MathUtil.applyDeadband(
    //             -m_translator.getX() * OperatorConstants.translationMultiplier,
    //             OperatorConstants.kDriveDeadband),
    //         () -> m_turnPower));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    CommandScheduler.getInstance()
        .schedule(
            DriveCommands.joystickDrive(
                m_drivetrain,
                () -> 0.0,  // X translation (stopped)
                () -> 0.0,  // Y translation (stopped)
                () -> 0.0  // Rotation (stopped)
            ));
  }

// public void continueDriving() {
//   CommandScheduler.getInstance()
//       .schedule(
//           DriveCommands.joystickDrive(
//               m_drivetrain,
//               () -> -m_translator.getY() * OperatorConstants.translationMultiplier,
//               () -> -m_translator.getX() * OperatorConstants.translationMultiplier,
//               () -> m_translator.getTwist() * OperatorConstants.rotationMultiplier
//           ));
// }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // End command when turn error is within tolerance
    return Math.abs(m_turnError) < Constants.VisionConstants.aprilTagAlignTolerance;
  }
}
