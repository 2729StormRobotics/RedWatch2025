package frc.robot.util.drive;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder.TriFunction;

public class DriveControls {
  // Controllers
  public static final CommandJoystick m_translator = new CommandJoystick(1);
  public static final CommandJoystick m_rotator = new CommandJoystick(2);
  public static final CommandXboxController m_weaponsController = new CommandXboxController(0);

  // Misc Subsystem Controls
  public static Trigger ROTATECLOCKWISE;
  public static Trigger ROTATECOUNTERCLOCKWISE;
  public static Trigger ARMSTOP;
  public static Trigger CALIBRATEARM;

  public static Trigger INTAKE;
  public static Trigger OUTTAKE;
  public static Trigger GRIPPERSTOP;

  // Useful for things that don't need to be triggered
  private static final Trigger EMPTY_TRIGGER = new Trigger(() -> false);
  private static final DoubleSupplier EMPTY_DOUBLE_SUPPLIER = () -> 0.0;


  // Main Button Controls
  public static Trigger L1;
  public static Trigger L2;
  public static Trigger L3;
  public static Trigger L4;

  // Drive controls
  public static DoubleSupplier ELEVATOR_JOYSTICK;
  public static DoubleSupplier PIVOT_ROTATE;
  public static Trigger PIVOT_L1_Intake;
  public static Trigger PIVOT_L2_Intake;
  public static Trigger PIVOT_L3_Intake;
  public static Trigger PIVOT_L4_Intake;

  // Drive controls
  public static DoubleSupplier DRIVE_FORWARD;
  public static DoubleSupplier DRIVE_STRAFE;
  public static DoubleSupplier DRIVE_ROTATE;

  public static Trigger DRIVE_SLOW;
  public static Trigger DRIVE_STOP;
  public static Trigger DRIVE_HOLD_STOP;

  // drive modes
  public static Trigger DRIVE_ROBOT_RELATIVE;
  public static Trigger DRIVE_FIELD_RELATIVE;
  public static Trigger DRIVE_REEF_AIM;

  // Drive Angle Locks
  public static Trigger LOCK_BACK;
  public static Trigger LOCK_PICKUP;
  public static Trigger LOCK_PASS;
  public static Trigger LOCK_ON_AMP;

  // Drive Trajectories
  public static Trigger DRIVE_R1;
  public static Trigger DRIVE_R2;
  public static Trigger DRIVE_R3;
  public static Trigger DRIVE_R4;
  public static Trigger DRIVE_R5;
  public static Trigger DRIVE_R6;

  public static Trigger DRIVE_PROCESSOR;
  public static Trigger DRIVE_FEED;

  public static Trigger MELTDOWN;

  public static Trigger RESET_GYRO;
  // SYSID Controls
  public static Trigger QUASISTATIC_FORWARD;
  public static Trigger QUASISTATIC_REVERSE;
  public static Trigger DYNAMIC_FORWARD;
  public static Trigger DYNAMIC_REVERSE;

  public static Trigger ELEVATOR_L1;
  public static Trigger ELEVATOR_L2;
  public static Trigger ELEVATOR_L3;
  public static Trigger ELEVATOR_L4;
  public static Trigger ELEVATOR_INTAKE;

  // Setup the controls
  public static void configureControls() {
    switch (Constants.driver) {
      case KRITHIK:
        // Driver controls
        DRIVE_FORWARD = () -> (-m_translator.getY());
        DRIVE_STRAFE = () -> (-m_translator.getX());
        DRIVE_ROTATE = () -> (-m_translator.getTwist());
        RESET_GYRO = m_translator.button(12);



        // Misc Subsytem Controls
        ROTATECLOCKWISE = m_weaponsController.rightBumper();
        ROTATECOUNTERCLOCKWISE = m_weaponsController.leftBumper();
        ARMSTOP = m_weaponsController.y();
        CALIBRATEARM = m_weaponsController.rightStick();

        INTAKE = m_weaponsController.rightBumper();
        OUTTAKE = m_weaponsController.leftBumper();
        GRIPPERSTOP = m_weaponsController.start();

        // Driver Settings
        DRIVE_SLOW = m_translator.button(1); // TBA
        DRIVE_STOP = m_translator.button(2); // TBA
        DRIVE_HOLD_STOP = m_translator.button(3); // TBA

        // Driver Modes
        DRIVE_ROBOT_RELATIVE = m_translator.button(4); // TBA
        DRIVE_REEF_AIM = m_translator.button(1);

        // ALL BELOW TBD
        // Driver Angle Locks
        LOCK_BACK = m_translator.button(1);
        LOCK_PICKUP = m_translator.button(1);
        LOCK_ON_AMP = m_translator.button(1);
        LOCK_PASS = m_translator.button(1); // uses vision

        // DRIVE_AMP = EMPTY_TRIGGER; // uses vision
        break;

      case PROGRAMMERS:
      default:
        // Driver controls
        DRIVE_FORWARD = () -> (-m_translator.getY());
        DRIVE_STRAFE = () -> (-m_translator.getX());
        DRIVE_ROTATE = () -> (-m_translator.getTwist());
        RESET_GYRO = m_translator.button(12);

        // Misc Subsytem Controls
        ROTATECLOCKWISE = m_weaponsController.rightBumper();
        ROTATECOUNTERCLOCKWISE = m_weaponsController.leftBumper();
        ARMSTOP = m_weaponsController.x();
        CALIBRATEARM = m_weaponsController.rightStick();

        INTAKE = m_weaponsController.a();
        OUTTAKE = m_weaponsController.b();
        GRIPPERSTOP = m_weaponsController.x();

        // Driver Settings
        DRIVE_SLOW = m_translator.button(1); // TBA
        DRIVE_STOP = m_translator.button(2); // TBA
        DRIVE_HOLD_STOP = m_translator.button(3); // TBA

        // Driver Modes
        DRIVE_ROBOT_RELATIVE = m_translator.button(4); // TBA
        // DRIVE_SPEAKER_AIM = m_translator.leftBumper(); // uses vision

        // ALL BELOW TBD
        // Driver Angle Locks
        LOCK_BACK = m_translator.button(1);
        LOCK_PICKUP = m_translator.button(1);
        LOCK_ON_AMP = m_translator.button(1);
        LOCK_PASS = m_translator.button(1); // uses vision

        // DRIVE_AMP = EMPTY_TRIGGER; // uses vision
    }

    switch (Constants.operator) {
      case KRITHIK:
      ELEVATOR_JOYSTICK = () -> (-m_weaponsController.getLeftY()/3);

        PIVOT_ROTATE = () -> (-m_weaponsController.getRightY() / 10);
        // all tbd
        // Pivot things
        L1 = m_weaponsController.b();
        L2 = m_weaponsController.a();
        L3 = m_weaponsController.x();
        L4 = m_weaponsController.y();

        // Misc Subsytem Controls
        ROTATECLOCKWISE = m_weaponsController.rightBumper();
        ROTATECOUNTERCLOCKWISE = m_weaponsController.leftBumper();
        ARMSTOP = m_weaponsController.x();
        INTAKE = m_weaponsController.a();
        OUTTAKE = m_weaponsController.b();
        GRIPPERSTOP = m_weaponsController.x();

        break;
      case PROGRAMMERS:
      default:
        // Operator controls
        PIVOT_ROTATE = () -> (m_weaponsController.getRightY());

        // ALL TBD

        // isn't reading m_weaponsController.getLeftTriggerAxis, must be an issue with
        // the encoder

        // Misc Subsytem Controls
        ROTATECLOCKWISE = m_weaponsController.rightBumper();
        ROTATECOUNTERCLOCKWISE = m_weaponsController.leftBumper();
        CALIBRATEARM = m_weaponsController.rightStick();
        ARMSTOP = m_weaponsController.x();

        INTAKE = m_weaponsController.a();
        OUTTAKE = m_weaponsController.b();
        GRIPPERSTOP = m_weaponsController.x();

        // bottom right Left joystick to intake
    }
  }
}
