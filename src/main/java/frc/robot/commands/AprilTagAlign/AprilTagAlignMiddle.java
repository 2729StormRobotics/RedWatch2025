package frc.robot.commands.AprilTagAlign;

//CAG:step one I'm blindly removing the yellow lines unless they throw an error just for readability.
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.PhotonVision.VisionConstants;
import frc.robot.subsystems.PhotonVision.VisionIO;

import java.util.function.DoubleSupplier;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * Command to align the robot with the nearest AprilTag using PhotonVision while
 * still allowing translational driving.
 */

public class AprilTagAlignMiddle extends Command implements VisionIO {
  private final Drive m_drivetrain;
  private final PIDController m_controller;
  private final PhotonCamera camera1;
  private boolean seeingTargets = false;
  private Rotation2d rotation;
  private PIDController pidController;
  private double targetAngle;

  // CAG: Looks like the x/y suppliers aren't currently required?
  public AprilTagAlignMiddle(Joystick joystick, Drive drivetrain,
      DoubleSupplier x_Supplier,
      DoubleSupplier y_Supplier) {
    m_drivetrain = drivetrain;
    // CAG: I'm surprised there's only one controller and it's only for turning.
    m_controller = new PIDController(Constants.VisionConstants.kPTurn, Constants.VisionConstants.kITurn,
        Constants.VisionConstants.kDTurn);
    camera1 = new PhotonCamera(VisionConstants.outtake_Cam);

    // CAG: What do we need this for now?
    this.rotation = drivetrain.getRotation();
    targetAngle = 0;
    // Initialize PID controller (tune these values as needed)

    // CAG: now we've got an m_controller and a this.pidController
    // It's a little weird to reference the same kind of thing two different ways
    // I'm more concerned about the name not being especially descriptive and that
    // there's only one.
    // At a glance, it seems like there's two different controllers for turning.
    // Assuming that's the case, having two is super confusing/risky.
    this.pidController = new PIDController(0.006, 0.0, 0.0);
    // CAG: The quick description I got was that all navigation is robot relative
    // once you see a tag
    // Assuming that's the case, a need for continuous input scares me.
    pidController.enableContinuousInput(-180, 180);
    pidController.setTolerance(1); // Allowable error in degrees

    addRequirements(m_drivetrain);
  }

  @Override
  public void initialize() {
    m_controller.reset();
    SmartDashboard.putString("AprilTagAlign", "Initialized");
  }

  @Override
  public void execute() {
    // CAG: the first get rotation seems unnecssary -
    // it seems like you don't do anything with it and then immediately ask for a
    // newer one.
    rotation = m_drivetrain.getRotation();
    // Read in relevant data from the Camera
    boolean targetVisible = false;
    double targetYaw = 0.0;
    double targetRange = 0.0;
    double yawThreshold = 1;
    double forwardMultiplier = 4.0;

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

      // CAG: Couple questions/thoughts here:
      // 1) This looks like it's going to be pose/field-oriented rather than robot
      // relative.
      // I'm not sure it's a great idea to mix and match those if you use the
      // targetRange/Yaw down the line.
      // targetYaw depends on every component of your pose, targetRange depends on two
      // The system gets complicated quickly if you're controlling measurements that
      // depend on multiple controller outputs.
      // 2) How does this behave if you can see multiple tags and the "best target"
      // changes?
      // 3) Do you have to do this every loop? Seems like you could set the target
      // angle/fiducial ID
      // In the constructor/init rather than doing it iteratively.
      // Then just check that the fiducialID of the primary target hasn't changed?
      SmartDashboard.putNumber("id number", target.getFiducialId());
      // FIND ANGLE FROM ID
      switch (target.getFiducialId()) {
        // BLUE SIDE TARGETS 18 is closet to DS
        case 22:
          targetAngle = -30;
          seeingTargets = true;
          break;
        case 21:
          targetAngle = -90;
          seeingTargets = true;
          break;
        case 20:
          targetAngle = -150;
          seeingTargets = true;
          break;
        case 19:
          targetAngle = 150;
          seeingTargets = true;
          break;
        case 18:
          targetAngle = 90;
          seeingTargets = true;
          break;
        case 17:
          targetAngle = 90;
          seeingTargets = true;
          break;
        // RED SIDE TARGERTS 9 -- 22
        case 9:
          targetAngle = -30;
          seeingTargets = true;
          break;
        case 10:
          targetAngle = -90;
          seeingTargets = true;
          break;
        case 11:
          targetAngle = -150;
          seeingTargets = true;
          break;
        case 6:
          targetAngle = 150;
          seeingTargets = true;
          break;
        case 7:
          targetAngle = 90;
          seeingTargets = true;
          break;
        case 8:
          targetAngle = 90;
          seeingTargets = true;
          break;
        default:
          seeingTargets = false;
          // code block
          targetAngle = 0;
      }
      // CAG: Why is the setpoint -targetAngle? Seems like this might be hiding a
      // coordinate frame problem.
      // CAG: Why do we need seeingTargets and targetVisible and results.hasTargets()?
      pidController.setSetpoint(-targetAngle);
      targetVisible = true;
    }

    // OFFSET PRESETS FOR DIFF OBJECTS (THIS IS LEFT)
    // CAG: Curious why we're adding offsets if everything is robot relative.
    // If everything is robot relative, you should be trying to get to the exact
    // same pose relative to the target every time.
    // I'm not sure you need to calculate that rather than just measuring it/polling
    // the camera....
    double cameraToArmOffset = Units.inchesToMeters(6); // add when going right,
    // subtract when going left
    double angleOffset = Units.radiansToDegrees(Math.atan((cameraToArmOffset) / targetRange)); // use trig to calculate
                                                                                               // angle
//CAG: This is super confusing. The target yaw is a robot relative measurement, I'm not sure why we'd shift it this way.
//If we shift something, I'd think it would be the setpoint
//but even then we can just measure the robot-relative yaw when the position is correct and the offset should be included.
    targetYaw -= angleOffset; // subtract offset to go right add to go left
    // Override the driver's turn command with an automatic one that turns toward
    // the tag.

    //CAG: These look like things that should be controllers.
    //They're similar to controllers, but doing a lot of the math in roundabout/implicit ways instead of directly.
    //I think there's a bunch of assumptions baked in here that may not always work out well.
    //It's probably cleaner to just have three actual controllers, and to try to get them on independent variables.
    // (X, Y, angle) in the same coordinate frame instead of different coordinate frames.
    double lateralSpeed = -0.25 * targetYaw * Constants.VisionConstants.kPTurn
        * DriveConstants.kMaxSpeedMetersPerSecond;
    double forward = forwardMultiplier * targetRange * Constants.VisionConstants.kPStrafe
        * DriveConstants.kMaxSpeedMetersPerSecond;
    // Put debug information to the dashboard
    SmartDashboard.putBoolean("Vision Target Visible", targetVisible);
    SmartDashboard.putNumber("Target Yaw", targetYaw);
    SmartDashboard.putNumber("Target range", targetRange);
    // Command drivetrain motors based on target speeds

    // Get the current gyro angle
    //CAG: you can just do this in one step back on line 80ish, right?
    // currentAngle = m_drivetrain.getRotation().getDegrees() or something.
    double currentAngle = rotation.getDegrees();

    // Calculate the rotation speed using the PID controller
    double rotationSpeed = pidController.calculate(currentAngle);
    SmartDashboard.putNumber("currangle", currentAngle);
    SmartDashboard.putNumber("desiangle", targetAngle);
    // Limit the speed to prevent over-rotation

    rotationSpeed = Math.max(-0.3, Math.min(0.3, rotationSpeed));
    SmartDashboard.putNumber("rotationspeed", rotationSpeed);
    // Convert to robot relative speeds & send command
    SmartDashboard.putBoolean("seeingtargets?", results.hasTargets());
    m_drivetrain.runVelocity(
        ChassisSpeeds.fromRobotRelativeSpeeds(
            lateralSpeed * m_drivetrain.getMaxLinearSpeedMetersPerSec(),
            forward * m_drivetrain.getMaxLinearSpeedMetersPerSec(),
            m_drivetrain.getMaxAngularSpeedRadPerSec() * rotationSpeed,
            // m_drivetrain.getMaxAngularSpeedRadPerSec()*0,
            new Rotation2d()));
            //CAG: does it matter what the rotation is here if you're tasking speeds?
            //If the answer is no, why is it an input?
            //CAG: It does matter - because the rotation is used to convert to field-centric driving.

    // if (targetYaw > 0) {
    // m_drivetrain.runVelocity(
    // ChassisSpeeds.fromRobotRelativeSpeeds(
    // m_drivetrain.getMaxLinearSpeedMetersPerSec() / 10,
    // 0,
    // 0,
    // new Rotation2d()));

    //CAG: This scares me - yaw depends on too many things to do this, I think.
    if (Math.abs(targetYaw) < yawThreshold) {
      lateralSpeed = 0;
    }
    seeingTargets = !(targetRange > -1.8);
  }

  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putString("AprilTagAlign", interrupted ? "Interrupted" : "Completed");
  }

  @Override
  public boolean isFinished() {
    SmartDashboard.putBoolean("end bool", (m_controller.atSetpoint()) || !seeingTargets);

    //CAG: I think this is probably a huge issue.
    // This can end if only one of your three dimensions is at its setpoint, which I don't think is desired.
    //I'm not sure I understand when seeingTargets is true/false or what it represents -
    //There's a few too many things that are named similarly and not many comments/explanations.
    // End command when turn error is within tolerance
    return (m_controller.atSetpoint()) || !seeingTargets;
  }
}
