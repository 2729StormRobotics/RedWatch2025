package frc.robot.commands.AprilTagAlign;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import java.util.function.DoubleSupplier;

/**
 * Command to align the robot with the nearest AprilTag using PhotonVision while
 * still allowing translational driving.
 */
public class AprilTagAlignTest extends Command {
    private final Drive m_drivetrain;
    private final PhotonCamera camera1;
    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;
    private final DoubleSupplier omegaSupplier;
    private boolean seeingTargets = false;
    private Rotation2d rotation;
    private final PIDController pidController; // Yaw PID
    private final PIDController lateralPID; // Lateral PID
    private double targetAngle;
    private double forward = 0;
    private final boolean isRightBranch; // Added parameter for branch selection
    private static final double YAW_THRESHOLD = 1;
    private static final double FORWARD_MULTIPLIER = 4.0;
    private static final double BRANCH_OFFSET_METERS = Units.inchesToMeters(6);
    private static final double ARM_OFFSET_METERS = Units.inchesToMeters(6);
    private static final double FIN_DISTANCE_REEF = 1.7;
    private static final double SLOW_DOWN_DISTANCE = Units.inchesToMeters(6); // Distance to start slowing down
    private static final double MIN_FORWARD_SPEED = 0.1; // Minimum forward speed
    private static final double MAX_FORWARD_SPEED = 0.5 * DriveConstants.kMaxSpeedMetersPerSecond;
    private static final double MAX_ROTATION_SPEED = 0.3;
    private static final double LATERAL_P = 0.05; // Tune these
    private static final double LATERAL_I = 0.0;
    private static final double LATERAL_D = 0.01;
    private static final double LATERAL_TOLERANCE = 0.05;
    private static final double FORWARD_FEEDFORWARD_GAIN = 0.1; // Example gain
    private static final double ROTATION_FEEDFORWARD_GAIN = 0.1; // Example gain
    private static final double LOST_TARGET_TIMEOUT = 0.5; //seconds
    private static final double DEADBAND = 0.02;

    // Dynamic PID Tuning Constants (Example values, tune these)
    private static final double YAW_KP_FAR = 0.1;
    private static final double YAW_KP_CLOSE = 0.05;
    private static final double YAW_KD_FAR = 0.02;
    private static final double YAW_KD_CLOSE = 0.01;
    private static final double LATERAL_KP_FAR = 0.1;
    private static final double LATERAL_KP_CLOSE = 0.05;
    private static final double LATERAL_KD_FAR = 0.02;
    private static final double LATERAL_KD_CLOSE = 0.01;
    private static final double DYNAMIC_TUNING_DISTANCE_THRESHOLD = 1.0; // Meters

    // State Machine
    private enum AlignmentState {
        SEARCHING,
        APPROACHING,
        ALIGNING,
        FINAL_ADJUSTMENT,
        ALIGNED
    }

    private AlignmentState currentState = AlignmentState.SEARCHING;
    private Timer stateTimer = new Timer();
    private Timer lostTargetTimer = new Timer(); // Timer to track lost target time
    private Pose3d lastSeenTagPose = new Pose3d(); // Store the last seen tag pose

    // Use a map for fiducial IDs to target angles
    private static final Map<Integer, Double> tagAngles = new HashMap<>();

    /**
     * Constructor for AprilTagAlignTest.
     *
     * @param drivetrain    The drive subsystem.
     * @param x_Supplier    Supplier for X-axis drive input.
     * @param y_Supplier    Supplier for Y-axis drive input.
     * @param omega_Supplier    Supplier for rotantional drive input.
     * @param isRightBranch Boolean indicating whether it's the right branch (true)
     * or left branch (false).
     */
    public AprilTagAlignTest(Drive drivetrain, DoubleSupplier x_Supplier, DoubleSupplier y_Supplier, DoubleSupplier omega_Supplier, boolean isRightBranch) {
        m_drivetrain = drivetrain;
        xSupplier = x_Supplier;
        ySupplier = y_Supplier;
        omegaSupplier = omega_Supplier;
        camera1 = new PhotonCamera(frc.robot.subsystems.Vision.VisionConstants.outtake_Cam);
        this.rotation = drivetrain.getRotation();
        this.targetAngle = 0;
        this.isRightBranch = isRightBranch; // Store the branch selection

        // Initialize the tagAngles map.  Using put() for more than 10 elements.
        tagAngles.put(22, -30.0);
        tagAngles.put(21, -90.0);
        tagAngles.put(20, -150.0);
        tagAngles.put(19, 150.0);
        tagAngles.put(18, 90.0);
        tagAngles.put(17, 30.0);
        tagAngles.put(9, -30.0);
        tagAngles.put(10, -90.0);
        tagAngles.put(11, -150.0);
        tagAngles.put(6, 150.0);
        tagAngles.put(7, 90.0);
        tagAngles.put(8, 30.0);

        // Initialize PID controller for angular alignment
        this.pidController = new PIDController(
                Constants.VisionConstants.kPTurn,
                Constants.VisionConstants.kITurn,
                Constants.VisionConstants.kDTurn);
        pidController.enableContinuousInput(-180, 180);
        pidController.setTolerance(1); // Allowable error in degrees

        // Initialize lateral PID controller
        this.lateralPID = new PIDController(LATERAL_P, LATERAL_I, LATERAL_D);
        lateralPID.setTolerance(LATERAL_TOLERANCE);

        addRequirements(m_drivetrain);
    }

    @Override
    public void initialize() {
        pidController.reset();
        lateralPID.reset();
        stateTimer.reset();
        stateTimer.start();
        lostTargetTimer.reset(); // Initialize lost target timer
        currentState = AlignmentState.SEARCHING;
        SmartDashboard.putString("AprilTagAlign", "Initialized");
    }

    private PhotonTrackedTarget getOptimalTarget(PhotonPipelineResult results) {
        Optional<PhotonTrackedTarget> target = results.getTargets().stream()
                .filter(t -> tagAngles.containsKey(t.getFiducialId())) // Only consider known tags
                .min((t1, t2) -> Double.compare(
                        t1.getBestCameraToTarget().getX(),
                        t2.getBestCameraToTarget().getX())); // Get closest tag
        if (target.isPresent()) {
            return target.get();
        } else {
            return null;
        }
    }

    private double calculateForwardSpeed(double targetRange) {
        double minSpeed = MIN_FORWARD_SPEED;
        double maxSpeed = FORWARD_MULTIPLIER * DriveConstants.kMaxSpeedMetersPerSecond;
        double decayFactor = 2.0; // Adjust to tune speed decay
        return Math.max(minSpeed, maxSpeed * Math.exp(-decayFactor * targetRange));
    }

    private void processTarget(PhotonTrackedTarget target) {
        SmartDashboard.putNumber("Optimal Target ID", target.getFiducialId());

        // Get target angle from ID, default to 0 and set seeingTargets
        targetAngle = tagAngles.getOrDefault(target.getFiducialId(), 0.0);
        seeingTargets = true;
        pidController.setSetpoint(-targetAngle);

        // Get the tag pose relative to the camera
        Transform3d cameraToTag = target.getBestCameraToTarget();
        Pose3d cameraPose = new Pose3d(); // changed
        Pose3d tagPoseInCamera = cameraPose.transformBy(cameraToTag);
        lastSeenTagPose = tagPoseInCamera; // Store the tag pose

        // Use the isRightBranch parameter here
        double netOffsetY = isRightBranch ? ARM_OFFSET_METERS + BRANCH_OFFSET_METERS : BRANCH_OFFSET_METERS - ARM_OFFSET_METERS;

        // Calculate branch offset
        Transform3d branchOffset = new Transform3d(
                new Translation3d(0.0, netOffsetY, 0.0),
                new Rotation3d());

        Pose3d branchPoseInCamera = tagPoseInCamera.transformBy(branchOffset);
        Transform3d cameraToBranch = new Transform3d(branchPoseInCamera.getTranslation(), branchPoseInCamera.getRotation());

        double branchYaw = Math.toDegrees(Math.atan2(cameraToBranch.getY(), cameraToBranch.getX()));
        double targetRange = cameraToBranch.getX();

        // Dynamic PID Tuning
        if (targetRange > DYNAMIC_TUNING_DISTANCE_THRESHOLD) {
            pidController.setP(YAW_KP_FAR);
            pidController.setD(YAW_KD_FAR);
            lateralPID.setP(LATERAL_KP_FAR);
            lateralPID.setD(LATERAL_KD_FAR);
        } else {
            pidController.setP(YAW_KP_CLOSE);
            pidController.setD(YAW_KD_CLOSE);
            lateralPID.setP(LATERAL_KP_CLOSE);
            lateralPID.setD(LATERAL_KD_CLOSE);
        }

        // Movement commands
        lateralPID.setSetpoint(0);
        double lateralSpeed = MathUtil.clamp(lateralPID.calculate(branchYaw, 0) + calculateLateralFeedforward(branchYaw), -DriveConstants.kMaxSpeedMetersPerSecond, DriveConstants.kMaxSpeedMetersPerSecond);
        forward = calculateForwardSpeed(targetRange) + calculateForwardFeedforward(targetRange);

        SmartDashboard.putNumber("Target Branch Yaw", branchYaw);
        SmartDashboard.putNumber("Target Branch Range", targetRange);
        SmartDashboard.putBoolean("Vision Target Visible", seeingTargets);
        lostTargetTimer.reset(); // Reset timer when target is seen
    }

    private double calculateLateralFeedforward(double yaw) {
        return ROTATION_FEEDFORWARD_GAIN * yaw;
    }

    private double calculateForwardFeedforward(double distance) {
        return FORWARD_FEEDFORWARD_GAIN * distance;
    }

    @Override
    public void execute() {
        rotation = m_drivetrain.getRotation();
        PhotonPipelineResult results = camera1.getLatestResult();
        double lateralSpeed = 0.0;
        forward = 0.0;

        switch (currentState) {
          case SEARCHING:
          // Basic search behavior, e.g., rotate slowly
          // Apply deadband
          double linearMagnitude = MathUtil.applyDeadband(
              Math.hypot(
                  xSupplier.getAsDouble(), ySupplier.getAsDouble()),
              DEADBAND);
          Rotation2d linearDirection = new Rotation2d(
              xSupplier.getAsDouble(), ySupplier.getAsDouble());
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);
  
          // Square values
          linearMagnitude = linearMagnitude * linearMagnitude;
          omega = Math.copySign(omega * omega, omega);
  
          // Calcaulate new linear velocity
          Translation2d linearVelocity = new Pose2d(new Translation2d(), linearDirection)
              .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
              .getTranslation();
  
          // Convert to field relative speeds & send command
          m_drivetrain.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  linearVelocity.getX() * m_drivetrain.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * m_drivetrain.getMaxLinearSpeedMetersPerSec(),
                  omega * m_drivetrain.getMaxAngularSpeedRadPerSec(),
                  m_drivetrain.getRotation().plus(new Rotation2d(Math.PI))));
  
                if (results.hasTargets() && results.getBestTarget() != null) { // found a target
                    currentState = AlignmentState.APPROACHING;
                    stateTimer.reset();
                    stateTimer.start();
                    lostTargetTimer.reset(); //reset
                }
                break;
            case APPROACHING:
                if (results.hasTargets() && results.getBestTarget() != null) {
                    PhotonTrackedTarget target = getOptimalTarget(results);
                    processTarget(target);
                    if (stateTimer.get() > 0.5) { // Transition to ALIGNING after a delay
                        currentState = AlignmentState.ALIGNING;
                        stateTimer.reset();
                        stateTimer.start();
                    }
                    lostTargetTimer.reset(); //reset
                } else {
                    //check if timer is greater than the timeout
                    if (lostTargetTimer.get() > LOST_TARGET_TIMEOUT) {
                        currentState = AlignmentState.SEARCHING; // lost target go back to searching
                        lostTargetTimer.reset();
                    } else {
                        // Drive to the last seen pose.
                        double lastSeenYaw = Math.toDegrees(Math.atan2(lastSeenTagPose.getY(), lastSeenTagPose.getX()));
                        double lastSeenRange = lastSeenTagPose.getX();

                        lateralSpeed = MathUtil.clamp(lateralPID.calculate(lastSeenYaw, 0) + calculateLateralFeedforward(lastSeenYaw), -DriveConstants.kMaxSpeedMetersPerSecond, DriveConstants.kMaxSpeedMetersPerSecond);
                        forward = calculateForwardSpeed(lastSeenRange) + calculateForwardFeedforward(lastSeenRange);
                    }
                }
                break;
            case ALIGNING:
                if (results.hasTargets() && results.getBestTarget() != null) {
                    PhotonTrackedTarget target = getOptimalTarget(results);
                    processTarget(target);
                    if (Math.abs(pidController.getPositionError()) < YAW_THRESHOLD
                            && Math.abs(lateralPID.getPositionError()) < LATERAL_TOLERANCE) {
                        currentState = AlignmentState.FINAL_ADJUSTMENT;
                        stateTimer.reset();
                        stateTimer.start();
                    }
                    lostTargetTimer.reset(); //reset
                } else {
                    if (lostTargetTimer.get() > LOST_TARGET_TIMEOUT) {
                        currentState = AlignmentState.SEARCHING; // lost target go back to searching
                        lostTargetTimer.reset();
                    } else {
                        double lastSeenYaw = Math.toDegrees(Math.atan2(lastSeenTagPose.getY(), lastSeenTagPose.getX()));
                        double lastSeenRange = lastSeenTagPose.getX();

                        lateralSpeed = MathUtil.clamp(lateralPID.calculate(lastSeenYaw, 0) + calculateLateralFeedforward(lastSeenYaw), -DriveConstants.kMaxSpeedMetersPerSecond, DriveConstants.kMaxSpeedMetersPerSecond);
                        forward = calculateForwardSpeed(lastSeenRange) + calculateForwardFeedforward(lastSeenRange);
                    }
                }
                break;
            case FINAL_ADJUSTMENT:
                if (results.hasTargets() && results.getBestTarget() != null) {
                    PhotonTrackedTarget target = getOptimalTarget(results);
                    processTarget(target);
                    if (stateTimer.get() > 0.2) {
                        currentState = AlignmentState.ALIGNED;
                        stateTimer.reset();
                        stateTimer.start();
                    }
                    lostTargetTimer.reset(); //reset
                } else {
                    if (lostTargetTimer.get() > LOST_TARGET_TIMEOUT) {
                        currentState = AlignmentState.SEARCHING; // lost target go back to searching
                        lostTargetTimer.reset();
                    } else {
                        double lastSeenYaw = Math.toDegrees(Math.atan2(lastSeenTagPose.getY(), lastSeenTagPose.getX()));
                        double lastSeenRange = lastSeenTagPose.getX();

                        lateralSpeed = MathUtil.clamp(lateralPID.calculate(lastSeenYaw, 0) + calculateLateralFeedforward(lastSeenYaw), -DriveConstants.kMaxSpeedMetersPerSecond, DriveConstants.kMaxSpeedMetersPerSecond);
                        forward = calculateForwardSpeed(lastSeenRange) + calculateForwardFeedforward(lastSeenRange);
                    }
                }
                break;
            case ALIGNED:
                // robot is aligned, drive = 0
                m_drivetrain.runVelocity(new ChassisSpeeds(0, 0, 0));
                break;
            default:
                currentState = AlignmentState.SEARCHING;
        }

        SmartDashboard.putString("Alignment State", currentState.toString());
        // Get the current gyro angle
        double currentAngle = rotation.getDegrees();

        // Calculate the rotation speed using the PID controller
        double rotationSpeed = MathUtil.clamp(pidController.calculate(currentAngle), -MAX_ROTATION_SPEED, MAX_ROTATION_SPEED);

        SmartDashboard.putNumber("currangle", currentAngle);
        SmartDashboard.putNumber("desiangle", targetAngle);
        SmartDashboard.putNumber("rotationspeed", rotationSpeed);

        // Drive the robot
        m_drivetrain.runVelocity(
                ChassisSpeeds.fromRobotRelativeSpeeds(
                        lateralSpeed * m_drivetrain.getMaxLinearSpeedMetersPerSec(),
                        forward * m_drivetrain.getMaxLinearSpeedMetersPerSec(),
                        m_drivetrain.getMaxAngularSpeedRadPerSec() * rotationSpeed,
                        new Rotation2d()));
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putString("AprilTagAlign", interrupted ? "Interrupted" : "Completed");
    }

    @Override
    public boolean isFinished() {
        // End command when turn error is within tolerance or no longer seeing targets
        return currentState == AlignmentState.ALIGNED;
    }
}

