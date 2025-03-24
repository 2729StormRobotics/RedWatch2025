package frc.robot.subsystems.arm;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.ArmIO.ArmIOInputs;
import frc.robot.subsystems.arm.ArmConstants;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class ArmIOSim implements ArmIO {
    private static final double LOOP_PERIOD_SECS = 0.02; //  Loop period for simulation updates.
    private static final DCMotor armMotorModel = DCMotor.getNEO(ArmConstants.kArmCANID); //  DCMotor model for the arm (NEO motor).

    public static final double armReduction = (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0); // Gear reduction ratio of the arm.

    private final ProfiledPIDController m_controller; //  PID controller for arm position control.
    private final DCMotorSim armSim = new DCMotorSim( //  DCMotor simulation for the arm.
            LinearSystemId.createDCMotorSystem(armMotorModel, 0.004, armReduction),
            armMotorModel);
    private ArmFeedforward m_feedforward = new ArmFeedforward(0, 0, 0, 0);

    // private final EncoderSim encoder = new EncoderSim(new
    // Encoder(DigitalSource(), null));
    // private final Encoder m_encoder;
    // public final EncoderSim m_encoderSim;
    private SingleJointedArmSim sim = new SingleJointedArmSim(
            armMotorModel,
            ArmConstants.ArmSimConstants.kArmReduction,
            SingleJointedArmSim.estimateMOI(ArmConstants.ArmSimConstants.kArmLength,
                    ArmConstants.ArmSimConstants.kArmMass),
            ArmConstants.ArmSimConstants.kArmLength,
            ArmConstants.ArmSimConstants.kMinAngleRads,
            ArmConstants.ArmSimConstants.kMaxAngleRads,
            true, // change this to true later.  Inverted?
            0.1);

    /**
     * Constructor for ArmIOSim.
     * Initializes the PID controller and encoder simulation.
     */
    public ArmIOSim() {
        // m_encoder = new Encoder(ArmConstants.ArmSimConstants.kEncoderAChannel,
        //         ArmConstants.ArmSimConstants.kEncoderBChannel);
        // m_encoderSim = new EncoderSim(m_encoder);
        // m_encoderSim.setDistancePerPulse(ArmConstants.ArmSimConstants.kArmEncoderDistPerPulse);
        m_controller = new ProfiledPIDController(ArmConstants.ArmSimConstants.kArmSimPID[0],
                ArmConstants.ArmSimConstants.kArmSimPID[1], ArmConstants.ArmSimConstants.kArmSimPID[2],
                new TrapezoidProfile.Constraints(2.45, 2.45)); //  Profiled PID Controller

        m_controller.setTolerance(0.1, 0.05); //  Set tolerance for position and velocity.
    }

    /**
     * Updates the inputs for the arm simulation.
     *
     * @param inputs The ArmIOInputs object to update.
     */
    public void updateInputs(ArmIOInputs inputs) {
        armSim.update(LOOP_PERIOD_SECS); //  Update the arm simulation.
        inputs.armAppliedVolts = getVoltage(); //  Get and set the applied voltage.
        inputs.armPositionRad = getArmAngleRad(); //  Get and set the arm position in radians.
        inputs.armPositionDegrees = getArmAngleDegrees(); // Get and set the arm position in degrees.
        inputs.armVelocityRadPerSec = getArmVelocity(); //  Get and set the arm velocity.
    }

    /**
     * Gets the voltage applied to the arm motor in the simulation.
     *
     * @return The applied voltage.
     */
    @Override
    public double getVoltage() {
        return armSim.getInputVoltage();
    }

    /**
     * Gets the current arm angle in radians from the simulation.
     *
     * @return The arm angle in radians.
     */
    @Override
    public double getArmAngleRad() {
        return sim.getAngleRads();
        // this is not right
    }

    /**
     * Gets the current arm angle in degrees from the simulation.
     *
     * @return The arm angle in degrees.
     */
    @Override
    public double getArmAngleDegrees() {
        return sim.getAngleRads() * (180 / Math.PI);
    }

    /**
     * Gets the current arm velocity in radians per second from the simulation.
     *
     * @return The arm velocity in radians per second.
     */
    @Override
    public double getArmVelocity() {
        return sim.getVelocityRadPerSec();
    }

    /**
     * Sets the desired arm position for the simulation.  This method uses a PID controller.
     *
     * @param kArmPositionPlaceholder The desired arm position.
     */
    public void setArmPosition(double kArmPositionPlaceholder) {
        m_controller.setGoal(kArmPositionPlaceholder); //  Set the PID controller's goal.
        // With the setpoint value we run PID control like normal
        double pidOutput = m_controller.calculate(getArmAngleDegrees()); // Calculate the PID output.
        double feedforwardOutput = m_feedforward.calculate(getArmAngleDegrees(),
                m_controller.getSetpoint().velocity);

        sim.setInputVoltage(feedforwardOutput + pidOutput); //  Set the input voltage to the arm simulation.
    }

    @Override
    public void setP(double p) {
        m_controller.setP(p);
    }

    @Override
    public void setI(double i) {
        m_controller.setI(i);
    }

    @Override
    public void setD(double d) {
        m_controller.setD(d);
    }

    @Override
    public void setkS(double kS) {
        m_feedforward = new ArmFeedforward(kS, m_feedforward.getKg(), m_feedforward.getKv(), m_feedforward.getKa());
    }

    @Override
    public void setkG(double kG) {
        m_feedforward = new ArmFeedforward(m_feedforward.getKs(), kG, m_feedforward.getKv(), m_feedforward.getKa());
    }

    @Override
    public void setkV(double kV) {
        m_feedforward = new ArmFeedforward(m_feedforward.getKs(), m_feedforward.getKg(), kV, m_feedforward.getKa());
    }

    @Override
    public void setkA(double kA) {
        m_feedforward = new ArmFeedforward(m_feedforward.getKs(), m_feedforward.getKg(), m_feedforward.getKv(), kA);
    }

    @Override
    public double getP() {
        return m_controller.getP();
    }

    @Override
    public double getI() {
        return m_controller.getI();
    }

    @Override
    public double getD() {
        return m_controller.getD();
    }

    @Override
    public double getkS() {
        return m_feedforward.getKs();
    }

    @Override
    public double getkG() {
        return m_feedforward.getKg();
    }

    @Override
    public double getkV() {
        return m_feedforward.getKv();
    }

    @Override
    public double getkA() {
        return m_feedforward.getKa();
    }
}
