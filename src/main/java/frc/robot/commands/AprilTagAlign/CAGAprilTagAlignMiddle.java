package frc.robot.commands.AprilTagAlign;

//CAG:step one I'm blindly removing the yellow lines unless they throw an error just for readability.
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.PhotonVision.VisionConstants;
import frc.robot.subsystems.PhotonVision.VisionIO;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * Command to align the robot with the nearest AprilTag using PhotonVision while
 * still allowing translational driving.
 */

public class CAGAprilTagAlignMiddle extends Command implements VisionIO {
  private final Drive m_drivetrain;
  private final PhotonCamera camera1;
  private PIDController pid_turn;
  private PIDController pid_x;
  private PIDController pid_y;
  private double targetX_m; // CAG: assuming the photon vision output is meters.
  private double targetY_m; // CAG: assuming the photon vision output is meters.
  private double targetYaw_deg; // CAG: assuming the photon vision output is degrees.

  // CAG: You could pass in the target, robot-relative X, Y, angle as arguemnts
  // Rather than having separate Left, Right, Middle commands. Separate commands
  // is clearer though.
  public CAGAprilTagAlignMiddle(Drive drivetrain) {
    m_drivetrain = drivetrain;
    // CAG: I'm surprised there's only one controller and it's only for turning.
    camera1 = new PhotonCamera(VisionConstants.outtake_Cam);

    // Initialize PID controller (tune these values as needed)
    // CAG: No idea what any of the constants/tolerances should be - that's a
    // testing problem.
    this.pid_turn = new PIDController(0, 0, 0);
    pid_turn.setTolerance(0);
    pid_turn.setSetpoint(targetYaw_deg); //CAG: have to actually/intelligently set/determine the targets.

    this.pid_x = new PIDController(0, 0, 0);
    pid_x.setTolerance(0);
    pid_x.setSetpoint(targetX_m);

    this.pid_y = new PIDController(0, 0, 0);
    pid_y.setTolerance(0);
    pid_y.setSetpoint(targetY_m);

    //CAG: Some notes about taking guesses at Kp, Ki, Kd:
    // proportional controller output = Kp * (target - setpoint)
    // So let's take yaw as an example - if we assume that the widest angle you can see the target at is 45 degrees
    // And that you're basically always trying to get to 0 degrees
    // Then max controller output = Kp * (45 - 0)
    // So if your controller output is a rotational speed, then you can pick what you think a reasonable speed is
    // If you think 100 degrees/sec is a good notional max speed when you're 45 degrees off:
    // Kp = 100 degrees/sec divided by 45 = 2.22 etc - you can bake in all of the constants you were using explicitly.
    // If you struggle to actually get to your target, Kp should go up until you get close enough/fast enough.
    // When Kp is too big, you'll overshoot and start oscillating.
    //Quick response and a little oscillation is generally an okay goal.
    // The sign of Kp determines the direction you turn in.

    //kI is generally important for holding precise setpoints over time.
    // Shouldn't be super important for things you're trying to do super quickly.

    //kD is used to dampen the oscllations you see either as a result of overshooting or disturbances.
    //Sign is super important for kD to make sure it dampens rather than exacerbates.


    addRequirements(m_drivetrain);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {

    PhotonPipelineResult results = camera1.getLatestResult();

    if (results.hasTargets()) {
      // At least one AprilTag was seen by the camera

      // CAG: you can do some kind of conditional check on whether
      // you see one of the reef tags here and break out early/set an end condition

      // CAG: You could also presumably lock a particular fiducial tag based on:
      // 1) what you see on init
      // 2) Your closest target based on pose
      // 3) A specific target based on a parent command
      // If you have a particular target you could presumably loop through all the
      // targets
      // and look for the one in question to avoid issues where you snap between
      // "best" targets.
      // I have no idea how necessary any of that is.

    }

    //CAG: I'm assuming we only get here if we have data from a relevant tag.

    // CAG: I'm not 100% that this works.
    // But I think this gives you X,Y,Z of the tag relative to the camera.
    // I'm not sure whether the target or the camera is the reference point, but I'm
    // not sure it matters.
    // If you can use these plus yaw, you have an indepdent X, Y, and angle that are
    // in the same coordinate frame.
    // Because all of these measurements are relative to the tag, they should be the
    // same at every reef point.
    // YOu should just be able to measure what the X, Y, and yaw are when you're in
    // a good spot.
    // Those should be the setpoints for every reef position.
    PhotonTrackedTarget target = results.getBestTarget();
    double currYaw = target.getYaw();
    Transform3d translation = target.getBestCameraToTarget();

    double currX = translation.getX();
    double currY = translation.getY();

    double rotationSpeed = pid_turn.calculate(currYaw);
    double xSpeed = pid_x.calculate(currX);
    double ySpeed = pid_y.calculate(currY);

    //CAG: I'm not sure this is necessary, but you can definitely do it.
    //Could also just tune PID output, but it can be safe to clamp the output.

//CAG: Probably worth tuning each controller independently first by hard-coding the tasked speeds to 0 for the other two.
//The x_controller should be able to get you to the targetX on its own. Same for Y and rotation.
//If they each do just their job and don't meaningfully impact the others, you should be okay to combine them.

    rotationSpeed = Math.max(-0.3, Math.min(0.3, rotationSpeed));
    m_drivetrain.runVelocity(
        ChassisSpeeds.fromRobotRelativeSpeeds(ySpeed,xSpeed,rotationSpeed,m_drivetrain.getRotation()));
  }

  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putString("AprilTagAlign", interrupted ? "Interrupted" : "Completed");
  }

  @Override
  public boolean isFinished() {

    //CAG: Check that all the controllers are happy before killing it.
    return (pid_turn.atSetpoint() && pid_x.atSetpoint() && pid_y.atSetpoint());
  }
}
