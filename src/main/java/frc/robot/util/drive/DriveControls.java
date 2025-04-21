package frc.robot.util.drive;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;

public class DriveControls {
  // Controllers
  public static final CommandJoystick m_translator = new CommandJoystick(1);
  public static final CommandJoystick m_rotator = new CommandJoystick(2);
  public static final CommandXboxController m_operator = new CommandXboxController(0);

  public static final CommandGenericHID m_sim_translator = new CommandGenericHID(0);
  public static final CommandGenericHID m_sim_rotator = new CommandGenericHID(1);
  public static final CommandGenericHID m_sim_operator = new CommandGenericHID(2);

  // Misc Subsystem Controls
  public static Trigger ROTATECLOCKWISE;
  public static Trigger ROTATECOUNTERCLOCKWISE;
  public static Trigger ARMSTOP;
  public static Trigger CALIBRATEARM;

  public static Trigger REVERSE;
  public static Trigger INTAKE;
  public static Trigger OUTTAKE;
  public static Trigger GRIPPERSTOP;

  // Main Button Controls
  public static Trigger STOW;
  public static Trigger L1;
  public static Trigger L2;
  public static Trigger L3;
  public static Trigger L4;
  public static Trigger AlgaeL2;
  public static Trigger AlgaeL3;
  public static Trigger INTAKE_POS;

  // Drive controls
  public static DoubleSupplier ELEVATOR_JOYSTICK;
  public static DoubleSupplier PIVOT_ROTATE;
  public static Trigger PULLHANGER;
  public static Trigger EXTENDHANGER;

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
  public static Trigger DRIVE_PHOTONVISION_ALIGN_RIGHT;
  public static Trigger DRIVE_PHOTONVISION_ALIGN_LEFT;
  public static Trigger DRIVE_PHOTONVISION_ALIGN_MIDDLE;

  // Drive Angle LocksDRIVE_PHOTONVISION_ALIGN_MIDDLE
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


  // POV Buttons
  public static POVButton DPAD_UP;
  public static POVButton DPAD_RIGHT;
  public static POVButton DPAD_DOWN;
  public static POVButton DPAD_LEFT;


  // Setup the controls
  public static void configureControls() {
    switch (Constants.mode) {
      case REAL:
          // REAL Driver controls
          // DRIVE_FORWARD = () -> (Math.copySign(Math.pow(((-m_translator.getY() * 4) / 4), 2), -m_translator.getY()));
          // DRIVE_STRAFE = () -> (Math.copySign(Math.pow(((-m_translator.getY() * 4) / 4), 2), -m_translator.getY()));
          
          DRIVE_FORWARD = () -> (Math.copySign(Math.pow(((-m_translator.getY() * 4) / 4), 2), -m_translator.getY()));
          DRIVE_STRAFE = () -> (Math.copySign(Math.pow(((-m_translator.getX() * 4) / 4), 2), -m_translator.getX()));
          DRIVE_ROTATE = () -> (((-m_rotator.getTwist() * 0.5)));
          RESET_GYRO = m_translator.button(12);
  
  
          // Driver Settings
          DRIVE_SLOW = m_translator.button(2); // TBA
          DRIVE_STOP = m_translator.button(2); // TBA
          DRIVE_HOLD_STOP = m_translator.button(3); // TBA
  
          // Driver Modes
          DRIVE_REEF_AIM = m_translator.button(1);
          DRIVE_PHOTONVISION_ALIGN_RIGHT = m_translator.button(6
          ); // uses vision right align
          DRIVE_PHOTONVISION_ALIGN_LEFT = m_translator.button(5); // uses vision left align
          DRIVE_PHOTONVISION_ALIGN_MIDDLE = m_translator.button(3);// middle vision align
  
          // ALL BELOW TBD
          // Driver Angle Locks
          LOCK_BACK = m_translator.button(1);
          LOCK_PICKUP = m_translator.button(1);
          LOCK_ON_AMP = m_translator.button(1);
          LOCK_PASS = m_translator.button(1); // uses vision
  
          // DRIVE_AMP = EMPTY_TRIGGER; // uses vision
          break;

      case SIM:

      // 
      // SIMULATION DRIVING
      // 
          // DRIVE_FORWARD = () -> (Math.copySign(Math.pow(((-m_translator.getY() * 4) / 4), 2), -m_translator.getY()));
          // DRIVE_STRAFE = () -> (Math.copySign(Math.pow(((-m_translator.getY() * 4) / 4), 2), -m_translator.getY()));
          
          DRIVE_FORWARD = () -> (Math.copySign(Math.pow(((-m_sim_translator.getRawAxis(0) * 4) / 4), 2), -m_sim_translator.getRawAxis(1)));
          DRIVE_STRAFE = () -> (Math.copySign(Math.pow(((-m_sim_translator.getRawAxis(1) * 4) / 4), 2), -m_sim_translator.getRawAxis(1)));
          DRIVE_ROTATE = () -> (((-m_rotator.getTwist() * 0.5)));
          RESET_GYRO = m_sim_translator.button(12);
  
  
          // Driver Settings
          DRIVE_SLOW = m_sim_translator.button(2); // TBA
          DRIVE_STOP = m_sim_translator.button(2); // TBA
          DRIVE_HOLD_STOP = m_sim_translator.button(3); // TBA
  
          // Driver Modes
          DRIVE_REEF_AIM = m_sim_translator.button(1);
          DRIVE_PHOTONVISION_ALIGN_RIGHT = m_sim_translator.button(6
          ); // uses vision right align
          DRIVE_PHOTONVISION_ALIGN_LEFT = m_sim_translator.button(5); // uses vision left align
          DRIVE_PHOTONVISION_ALIGN_MIDDLE = m_sim_translator.button(3);// middle vision align
  
          // ALL BELOW TBD
          // Driver Angle Locks
          LOCK_BACK = m_sim_translator.button(1);
          LOCK_PICKUP = m_sim_translator.button(1);
          LOCK_ON_AMP = m_sim_translator.button(1);
          LOCK_PASS = m_sim_translator.button(1); // uses vision
  
         
  
          REVERSE = m_sim_operator.button(0);
          INTAKE = m_sim_operator.button(3);
          OUTTAKE = m_sim_operator.button(3);
          GRIPPERSTOP = m_sim_operator.button(2);
          // ELEVATOR_JOYSTICK = () -> (-m_operator.getLeftY()/2);
          ELEVATOR_JOYSTICK = () -> (Math.copySign(Math.pow(((-m_sim_operator.getRawAxis(0) * 3) / 4), 2), -m_sim_operator.getRawAxis(0)));
  
          EXTENDHANGER = m_sim_operator.button(1);
          PULLHANGER = m_sim_operator.button(0);
  
          MELTDOWN = m_sim_operator.button(0);
          PIVOT_ROTATE = () -> (-m_sim_operator.getRawAxis(1) / 10);
          // all tbd
          // Pivot things
          STOW = m_sim_operator.button(0);
          L1 = m_sim_operator.povUp();
          L2 = m_sim_operator.povRight();
          L3 = m_sim_operator.povDown();
          L4 = m_sim_operator.povLeft();
          AlgaeL2 = m_sim_operator.button(2);
          AlgaeL3 = m_sim_operator.button(3);
          INTAKE_POS = m_sim_operator.button(4);
          // Misc Subsytem Controls
          ARMSTOP = m_sim_operator.button(5);
  
        
      
        break;
    
      default:
        break;
    }
    
  }
}
