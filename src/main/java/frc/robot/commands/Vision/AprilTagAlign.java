// package frc.robot.commands.Vision;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.CommandScheduler;
// import frc.robot.Constants;
// import frc.robot.LimelightHelpers;
// import frc.robot.Constants.OperatorConstants;
// import frc.robot.commands.DriveCommands;
// import frc.robot.subsystems.drive.Drive;
// import frc.robot.subsystems.Vision;

// /*
//  * Makes the robot automatically align to nearest apriltag (while still being able to drive translationally)
//  */
// public class AprilTagAlign extends Command {
//   private final Vision m_vision; 
//   private final Drive m_drivetrain;
//   private final Joystick m_translator;
//   private final PIDController m_controller;
//   private double m_turnError = 0.0;
//   private double m_turnPower;

//   /** Creates a new AprilTagAlign. */
//   public AprilTagAlign(Joystick joystick, Drive drivetrain) {
//     m_vision = Vision.getInstance();
//     m_drivetrain = drivetrain;
//     m_translator = joystick;
//     m_controller = new PIDController(Constants.VisionConstants.kPTurn, Constants.VisionConstants.kITurn, Constants.VisionConstants.kDTurn);
//     // Use addRequirements() here to declare subsystem dependencies.
//     addRequirements(m_drivetrain);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     // Reset PID controller and initialize values
//     m_controller.reset();
//     m_turnError = LimelightHelpers.getTX("limelight");
//     m_turnPower = 0;
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     while (Math.abs(m_turnError) > Constants.VisionConstants.aprilTagAlignTolerance) {
//       m_turnError = LimelightHelpers.getTX("limelight"); // Horizontal angle away from target
//       if (m_turnError > 0) {
//         m_turnPower = Constants.VisionConstants.kPTurn * m_turnError + Constants.VisionConstants.kSTurn;
//       } else {
//         m_turnPower = Constants.VisionConstants.kPTurn * m_turnError - Constants.VisionConstants.kSTurn;
//       }
//       // m_turnPower = m_controller.calculate(m_turnError); // Calculate PID output
//       // System.out.println("Turn Error: " + m_turnPower);
//       // Constrain turnPower to a safe range
//       m_turnPower = MathUtil.clamp(m_turnPower, -1.0, 1.0);

//       // Stop turning if error is within tolerance
//       if (Math.abs(m_turnError) < Constants.VisionConstants.aprilTagAlignTolerance) {
//         m_turnPower = 0.0;
//       }

//       // Send debug info to SmartDashboard
//       SmartDashboard.putNumber("turnError", m_turnError);
//       SmartDashboard.putNumber("turnPower", m_turnPower);

//       // Drive the robot
//       CommandScheduler.getInstance()
//           .schedule(
//               DriveCommands.joystickDrive(
//                   m_drivetrain,
//                   () -> MathUtil.applyDeadband(
//                       -m_translator.getY() * OperatorConstants.translationMultiplier,
//                       OperatorConstants.kDriveDeadband),
//                   () -> MathUtil.applyDeadband(
//                       -m_translator.getX() * OperatorConstants.translationMultiplier,
//                       OperatorConstants.kDriveDeadband),
//                   () -> m_turnPower));
//     }

//     // m_turnPower = 0.0;

//     // // Drive the robot
//     // CommandScheduler.getInstance()
//     // .schedule(
//     //     DriveCommands.joystickDrive(
//     //         m_drivetrain,
//     //         () -> MathUtil.applyDeadband(
//     //             -m_translator.getY() * OperatorConstants.translationMultiplier,
//     //             OperatorConstants.kDriveDeadband),
//     //         () -> MathUtil.applyDeadband(
//     //             -m_translator.getX() * OperatorConstants.translationMultiplier,
//     //             OperatorConstants.kDriveDeadband),
//     //         () -> m_turnPower));
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     CommandScheduler.getInstance()
//         .schedule(
//             DriveCommands.joystickDrive(
//                 m_drivetrain,
//                 () -> 0.0,  // X translation (stopped)
//                 () -> 0.0,  // Y translation (stopped)
//                 () -> 0.0  // Rotation (stopped)
//             ));
//   }

// // public void continueDriving() {
// //   CommandScheduler.getInstance()
// //       .schedule(
// //           DriveCommands.joystickDrive(
// //               m_drivetrain,
// //               () -> -m_translator.getY() * OperatorConstants.translationMultiplier,
// //               () -> -m_translator.getX() * OperatorConstants.translationMultiplier,
// //               () -> m_translator.getTwist() * OperatorConstants.rotationMultiplier
// //           ));
// // }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     // End command when turn error is within tolerance
//     return Math.abs(m_turnError) < Constants.VisionConstants.aprilTagAlignTolerance;
//   }
// }

package frc.robot.commands.Vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

/**
 * Makes the robot automatically align to the nearest AprilTag while allowing translational driving.
 */
public class AprilTagAlign extends Command {
  private final Vision m_vision;
  private final Drivetrain m_drivetrain;
  private final Joystick m_translator;
  private final PIDController m_controller;
  private double m_turnError;
  private double m_turnPower;

  /** Creates a new AprilTagAlign. */
  public AprilTagAlign(Joystick joystick) {
    m_vision = Vision.getInstance();
    m_drivetrain = Drivetrain.getInstance();
    m_translator = joystick;
    m_controller = new PIDController(
        Constants.VisionConstants.kPTurn,
        Constants.VisionConstants.kITurn,
        Constants.VisionConstants.kDTurn
    );

    // Set the tolerance for the PID controller
    m_controller.setTolerance(Constants.VisionConstants.aprilTagAlignTolerance);

    // Declare subsystem dependencies
    addRequirements(m_drivetrain);
  }

  @Override
  public void initialize() {
    // Reset PID values
    m_controller.reset();
    m_turnError = 0.0;
    m_turnPower = 0.0;

    // Optional: Log initialization
    SmartDashboard.putString("AprilTagAlign", "Initialized");
  }

  @Override
  public void execute() {
    // Cache the current vision angle
    m_turnError = m_vision.getX();

    // Calculate the turn power
    m_turnPower = m_controller.calculate(m_turnError) 
        + Math.copySign(Constants.VisionConstants.kSTurn, m_turnError);

    // If the error is within a very small range, stop turning
    if (Math.abs(m_turnError) < Constants.VisionConstants.aprilTagAlignTolerance) {
      m_turnPower = 0.0;
    }

    // Drive the robot with translational and rotational inputs
    m_drivetrain.drive(
        MathUtil.applyDeadband(
            -m_translator.getY() * OperatorConstants.translationMultiplier,
            OperatorConstants.kDriveDeadband
        ),
        MathUtil.applyDeadband(
            -m_translator.getX() * OperatorConstants.translationMultiplier,
            OperatorConstants.kDriveDeadband
        ),
        m_turnPower,
        true, true
    );

    // Send debugging information to SmartDashboard
    SmartDashboard.putNumber("Turn Error", m_turnError);
    SmartDashboard.putNumber("Turn Power", m_turnPower);
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the drivetrain
    m_drivetrain.drive(0.0, 0.0, 0.0, true, true);

    // Optional: Log termination
    SmartDashboard.putString("AprilTagAlign", interrupted ? "Interrupted" : "Completed");
  }

  @Override
  public boolean isFinished() {
    // Optionally use the PID controller's built-in tolerance check
    return m_controller.atSetpoint();
  }
}
