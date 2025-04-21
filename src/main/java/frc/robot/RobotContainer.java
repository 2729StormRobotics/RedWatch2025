// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static frc.robot.util.drive.DriveControls.*;

import frc.robot.commands.AprilTagAlign.AprilTagAlignTest;
import frc.robot.commands.AprilTagAlign.AprilTagAlignTestRight;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralAlgaeStack;
import static edu.wpi.first.units.Units.Inches;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.Mode;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.arm.ArmIOSparkMax;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOReal;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkMax;
import frc.robot.subsystems.elevator.ElevatorIOSparkFlex;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.gripper.GripperIOSim;
import frc.robot.subsystems.gripper.GripperIOSparkMax;
import frc.robot.util.drive.DriveControls;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
//~
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSIM;
import frc.robot.subsystems.hanger.*;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhoton;
import frc.robot.subsystems.vision.VisionIOSim;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Elevator elevator;
  private final Arm arm;
  private final Gripper m_gripper;
  private final HangerIO hanger;

  private Mechanism2d elevatorMech = new Mechanism2d(3, 3);
  private final DriveTrainSimulationConfig driveTrainSimulationConfig = DriveTrainSimulationConfig.Default()
      // Specify gyro type (for realistic gyro drifting and error simulation)
      .withGyro(COTS.ofPigeon2())
      // Specify swerve module (for realistic swerve dynamics)
      .withSwerveModule(COTS.ofMark4(
          DCMotor.getKrakenX60(1), // Drive motor is a Kraken X60
          DCMotor.getFalcon500(1), // Steer motor is a Falcon 500
          COTS.WHEELS.COLSONS.cof, // Use the COF for Colson Wheels
          3)) // L3 Gear ratio
      // Configures the track length and track width (spacing between swerve modules)
      .withTrackLengthTrackWidth(Inches.of(DriveConstants.kTrackWidthX), Inches.of(DriveConstants.kTrackWidthX))
      // Configures the bumper size (dimensions of the robot bumper)
      .withBumperSize(Inches.of(28), Inches.of(28));
  // LEDs
  private SwerveDriveSimulation swerveDriveSimulation = new SwerveDriveSimulation(
      // Specify Configuration
      driveTrainSimulationConfig,
      // Specify starting pose
      new Pose2d(3, 3, new Rotation2d()));
  // LEDs

  // Dashboard inputs
  private LoggedDashboardChooser<Command> autoChooser;
  // Field
  private final Field2d field;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations

        elevator = new Elevator(new ElevatorIOSparkFlex());
        drive = new Drive(
            new GyroIOReal(),
            new ModuleIOSparkMax(0),
            new ModuleIOSparkMax(1),
            new ModuleIOSparkMax(2),
            new ModuleIOSparkMax(3),
            new VisionIOPhoton());
        arm = new Arm(new ArmIOSparkMax());
        m_gripper = new Gripper(new GripperIOSparkMax());
        hanger = new HangerIOSparkMax();
        break;

      case SIM:
        SimulatedArena.getInstance().addDriveTrainSimulation(swerveDriveSimulation);
        // Sim robot, instantiate physics sim IO implementations
        elevator = new Elevator(new ElevatorIOSIM());
        drive = new Drive(
            new GyroIOSim(swerveDriveSimulation.getGyroSimulation()),
            new ModuleIOSim(swerveDriveSimulation.getModules()[0]),
            new ModuleIOSim(swerveDriveSimulation.getModules()[1]),
            new ModuleIOSim(swerveDriveSimulation.getModules()[2]),
            new ModuleIOSim(swerveDriveSimulation.getModules()[3]),
            new VisionIOSim());
        arm = new Arm(new ArmIOSim());
        m_gripper = new Gripper(new GripperIOSim());
        hanger = new HangerIOSim();

        break;

      default:
        // Replayed robot, disable IO implementations
        elevator = new Elevator(new ElevatorIO() {
        });

        drive = new Drive(
            new GyroIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            },
            new VisionIO() {
            });
        arm = new Arm(new ArmIO() {
        });
        m_gripper = new Gripper(new GripperIOSim());
        hanger = new HangerIOSparkMax();

        break;
    }

    NamedCommands.registerCommand("L2Setpoint",
        new SequentialCommandGroup(elevator.PIDCommand(ElevatorConstants.L4).withTimeout(1.2),
            // arm.PIDCommand(ArmConstants.kL4).withTimeout(0.45)));
            arm.PIDCommand(ArmConstants.kL4).withTimeout(0.5), m_gripper.outtake().withTimeout(0.5),
            new WaitCommand(0.4), arm.PIDCommand(ArmConstants.kSTOW).withTimeout(0.3)));

    NamedCommands.registerCommand("L4Setpoint",
        new SequentialCommandGroup(elevator.PIDCommand(ElevatorConstants.L4).withTimeout(1.5),
            arm.PIDCommand(ArmConstants.kL4).withTimeout(1)));
    NamedCommands.registerCommand("IntakeSetpoint",
        new ParallelCommandGroup(elevator.PIDCommand(ElevatorConstants.INTAKE).withTimeout(2),
            new SequentialCommandGroup(new WaitCommand(0), arm.PIDCommand(ArmConstants.kIntake).withTimeout(1))));
    NamedCommands.registerCommand("Intake", m_gripper.Intake().withTimeout(2));
    NamedCommands.registerCommand("Outtake", new WaitCommand(0));

    // NamedCommands.registerCommand("Outtake", new
    // ParallelDeadlineGroup(m_gripper.outtake(),
    // elevator.PIDCommand(ElevatorConstants.L4), new SequentialCommandGroup(new
    // WaitCommand(0.9), arm.PIDCommand(ArmConstants.kSTOW))));
    NamedCommands.registerCommand("AlignReefLeft", new AprilTagAlignTest(drive, () -> 0, () -> 0, () -> 0, false)
        .withTimeout(2.5).andThen(DriveCommands.joystickDrive(drive, () -> 0, () -> 0, () -> 0).withTimeout(0.01)));
    NamedCommands.registerCommand("AlignReefRight", new AprilTagAlignTestRight(drive, () -> 0, () -> 0, () -> 0, false)
        .withTimeout(2.5).andThen(DriveCommands.joystickDrive(drive, () -> 0, () -> 0, () -> 0).withTimeout(0.01)));
    NamedCommands.registerCommand("AngleReset", new InstantCommand(() -> {
      drive.resetYaw();
    }));
    NamedCommands.registerCommand("Algae3", new ParallelCommandGroup(elevator.PIDCommand(ElevatorConstants.AlgaeL2),
        new SequentialCommandGroup(new WaitCommand(1), arm.PIDCommand(ArmConstants.kAlgae)), m_gripper.reverse()));
    NamedCommands.registerCommand("Stow", // put the nail in the horsehoe
        new ParallelCommandGroup(elevator.PIDCommand(ElevatorConstants.L3).withTimeout(0.5),
            arm.PIDCommand(ArmConstants.kSTOW).withTimeout(0.5)));

    field = new Field2d();
    SmartDashboard.putData("Field", field);

    System.out.println("[Init] Setting up Path Planner Logging");

    // Logging callback for current robot pose
    PathPlannerLogging.setLogCurrentPoseCallback(
        (pose) -> {
          // Do whatever you want with the pose here
          field.setRobotPose(pose);
          Logger.recordOutput("PathPlanner/RobotPose", pose);
        });

    // Logging callback for target robot pose
    PathPlannerLogging.setLogTargetPoseCallback(
        (pose) -> {
          // Do whatever you want with the pose here
          field.getObject("target pose").setPose(pose);
          Logger.recordOutput("PathPlanner/TargetPose", pose);
        });

    // Logging callback for the active path, this is sent as a list of poses
    PathPlannerLogging.setLogActivePathCallback(
        (poses) -> {
          // Do whatever you want with the poses here
          field.getObject("path").setPoses(poses);
          Logger.recordOutput("PathPlanner/ActivePath", poses.toArray(new Pose2d[0]));
        });

    MechanismRoot2d elevatorRoot = elevatorMech.getRoot("elevator", 1, 0.5);
    elevatorRoot.append(elevator.getElevatorMechanism());
    // add subsystem mechanisms
    SmartDashboard.putData("Elevator Mechanism", elevatorMech);

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up Drive SysId routines
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Set up Named Commands

    configureButtonBindings();

    // Set up auto routines
    // System.out.println("[Init] Setting up Logged Auto Chooser");
    // autoChooser = new LoggedDashboardChooser<>("Auto Choices",
    // AutoBuilder.buildAutoChooser());
  }

  // zero gyro
  public void reset() {
    drive.resetYaw();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    DriveControls.configureControls();

    // Drive Commands
    SmartDashboard.putData("commandscheduler", CommandScheduler.getInstance());
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(drive, DRIVE_FORWARD, DRIVE_STRAFE, DRIVE_ROTATE));

    DRIVE_SLOW.onTrue(DriveCommands.joystickDrive(drive, DRIVE_FORWARD, DRIVE_STRAFE, DRIVE_ROTATE));
    // DRIVE_PHOTONVISION_ALIGN_RIGHT
    // .whileTrue(
    // new SequentialCommandGroup(new AprilTagAlignLeft( drive, DRIVE_FORWARD,
    // DRIVE_STRAFE),
    // DriveCommands.joystickDriveRobotRelative(drive, () -> -0.4, () -> -0.05, ()
    // -> 0).withTimeout(.51))
    // .withTimeout(20));
    // DRIVE_PHOTONVISION_ALIGN_LEFT
    // .whileTrue(
    // new SequentialCommandGroup(new AprilTagAlignLeft( drive, DRIVE_FORWARD,
    // DRIVE_STRAFE),
    // DriveCommands.joystickDriveRobotRelative(drive, () -> 0.4, () -> -0.05, () ->
    // 0).withTimeout(.1))
    // .withTimeout(20));
    DRIVE_PHOTONVISION_ALIGN_RIGHT.whileTrue(
        new ParallelCommandGroup(arm.PIDCommand(ArmConstants.kSTOW), elevator.PIDCommand(ElevatorConstants.L3),
            new AprilTagAlignTestRight(drive, DRIVE_FORWARD, DRIVE_STRAFE, DRIVE_ROTATE, false)));
    DRIVE_PHOTONVISION_ALIGN_LEFT
        .whileTrue(new AprilTagAlignTest(drive, DRIVE_FORWARD, DRIVE_STRAFE, DRIVE_ROTATE, false));

    // .onFalse(drive.getDefaultCommand());
    // DRIVE_PHOTONVISION_ALIGN_MIDDLE
    // .onTrue(
    // new SequentialCommandGroup(new AprilTagAlignMiddle(m_rotator.getHID(), drive,
    // DRIVE_FORWARD, DRIVE_STRAFE),
    // new ParallelDeadlineGroup(new WaitCommand(2),
    // DriveCommands.joystickDriveRobotRelative(drive, () -> 0.2, () -> 0, () ->
    // 0))))
    // .onFalse(drive.getDefaultCommand());

    RESET_GYRO.onTrue(
        new InstantCommand(
            () -> {
              drive.resetYaw();
            },
            drive));

    // DRIVE_ROBOT_RELATIVE.onTrue(DriveCommands.joystickDriveRobotRelative(drive,
    // DRIVE_STRAFE, DRIVE_FORWARD, DRIVE_ROTATE));
    SmartDashboard.putNumber("Elevator Joystick", ELEVATOR_JOYSTICK.getAsDouble());
    // Elevator Commands
    elevator.setDefaultCommand(elevator.ManualCommand(ELEVATOR_JOYSTICK));

    if (Constants.mode == Mode.SIM) {
      DriveControls.STOW.onTrue(new InstantCommand(() -> {
        SimulatedArena.getInstance().resetFieldForAuto();
        // SimulatedArena.getInstance().addGamePiece(new ReefscapeCoralAlgaeStack(new
        // Translation2d(2,2)));
      }));
      DriveControls.STOW.onTrue(new InstantCommand(() -> {
        // SimulatedArena.getInstance().resetFieldForAuto();
        SimulatedArena.getInstance().addGamePiece(new ReefscapeCoralAlgaeStack(new Translation2d(2, 2)));
      }));
    }

    MELTDOWN.onTrue(new SequentialCommandGroup(new InstantCommand(() -> {
      elevator.setVelocity(0);
    }, elevator), arm.stop(), m_gripper.stop(), new InstantCommand(() -> {
      hanger.stop();
    })));
    // Arm Commands
    arm.setDefaultCommand(arm.ManualCommand(PIVOT_ROTATE));
    // Gripper Commands
    // if (elevator.isLevelL4) {
    // OUTTAKE.onTrue(new ParallelCommandGroup(m_gripper.outtake(),new
    // SequentialCommandGroup(new WaitCommand(0.15),
    // arm.PIDCommand(ArmConstants.kSTOW)) ));

    // } else {
    // OUTTAKE.onTrue(new ParallelCommandGroup(m_gripper.outtake()));
    // }
    // OUTTAKE.onTrue(m_gripper.outtake());
    OUTTAKE.onTrue(new ConditionalCommand(
        new ParallelCommandGroup(elevator.PIDCommand(ElevatorConstants.L4), m_gripper.outtake(),
            new SequentialCommandGroup(new WaitCommand(0.40), arm.PIDCommand(ArmConstants.kSTOW))),
        m_gripper.outtake(), () -> elevator.getL4()));
    // OUTTAKE.onTrue(new ValidatedOuttake(elevator.getL4(), arm, m_gripper,
    // elevator));
    INTAKE.onTrue(m_gripper.Intake());
    GRIPPERSTOP.onTrue(m_gripper.stop());
    REVERSE.onTrue(m_gripper.reverse());

    PULLHANGER.whileTrue(new InstantCommand(() -> {
      hanger.pull();
    }));
    PULLHANGER.onFalse(new InstantCommand(() -> {
      hanger.stop();
    }));
    EXTENDHANGER.whileTrue(new InstantCommand(() -> {
      hanger.release();
    }));
    EXTENDHANGER.onFalse(new InstantCommand(() -> {
      hanger.stop();
    }));

    // Set Positions
    // DriveControls.L1.onTrue(arm.PIDCommand(32));
    // DriveControls.L2.onTrue(arm.PIDCommand(90));
    // DriveControls.L3.onTrue(arm.PIDCommand(120));
    // DriveControls.L1.onTrue(elevator.ManualCommand(0.05));

    // Real Set Positions
    DriveControls.STOW.onTrue(new ParallelCommandGroup(new InstantCommand(() -> {
      elevator.isLevelL4 = false;
    }), elevator.PIDCommand(ElevatorConstants.STOW),
        new SequentialCommandGroup(new WaitCommand(0), arm.PIDCommand(ArmConstants.kSTOW))));

    DriveControls.L1.onTrue(new ParallelCommandGroup(new InstantCommand(() -> {
      elevator.isLevelL4 = false;
    }), elevator.PIDCommand(ElevatorConstants.L1),
        new SequentialCommandGroup(new WaitCommand(0), arm.PIDCommand(ArmConstants.kL1))));

    DriveControls.L2.onTrue(new ParallelCommandGroup(new InstantCommand(() -> {
      elevator.isLevelL4 = false;
    }), elevator.PIDCommand(ElevatorConstants.L2),
        new SequentialCommandGroup(new WaitCommand(0.35), arm.PIDCommand(ArmConstants.kL2))));

    DriveControls.L3.onTrue(new ParallelCommandGroup(new InstantCommand(() -> elevator.isLevelL4 = false),
        elevator.PIDCommand(ElevatorConstants.L3),
        new SequentialCommandGroup(new WaitCommand(0.5), arm.PIDCommand(ArmConstants.kL3))));

    DriveControls.L4.onTrue(
        new SequentialCommandGroup(elevator.setL4(), new ParallelCommandGroup(elevator.PIDCommand(ElevatorConstants.L4),
            new SequentialCommandGroup(new WaitCommand(1), arm.PIDCommand(ArmConstants.kL4)))));

    DriveControls.AlgaeL2.onTrue(new ParallelCommandGroup(elevator.PIDCommand(ElevatorConstants.AlgaeL2),
        new SequentialCommandGroup(new WaitCommand(1), arm.PIDCommand(ArmConstants.kAlgae))));
    DriveControls.AlgaeL3.onTrue(new ParallelCommandGroup(elevator.PIDCommand(ElevatorConstants.AlgaeL3),
        new SequentialCommandGroup(new WaitCommand(1), arm.PIDCommand(ArmConstants.kAlgae))));

    DriveControls.INTAKE_POS.onTrue(new ParallelCommandGroup(elevator.PIDCommand(ElevatorConstants.INTAKE),
        new SequentialCommandGroup(new WaitCommand(0), arm.PIDCommand(ArmConstants.kIntake))));

    // Elevator only

    // DriveControls.L1.onTrue(elevator.PIDCommand(ElevatorConstants.L1));

    // DriveControls.L2.onTrue(elevator.PIDCommand(ElevatorConstants.L2));

    // DriveControls.L3.onTrue(elevator.PIDCommand(ElevatorConstants.L3));

    // DriveControls.L4.onTrue(elevator.PIDCommand(ElevatorConstants.L4));

    // DriveControls.INTAKE_POS.onTrue(elevator.PIDCommand(ElevatorConstants.INTAKE));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    System.out.println(autoChooser.get().getName());
    System.out.println();
    return autoChooser.get();
  }

  // Subsystem compound commands

  public Command goToL1() {
    return elevator.InstantPIDCommand(ElevatorConstants.L1)
        .alongWith(arm.InstantPIDCommand(ArmConstants.kL1));
  }

  public Command goToL2() {
    return elevator.InstantPIDCommand(ElevatorConstants.L2)
        .alongWith(arm.InstantPIDCommand(ArmConstants.kL2));
  }

  public Command goToL3() {
    return elevator.InstantPIDCommand(ElevatorConstants.L3)
        .alongWith(arm.InstantPIDCommand(ArmConstants.kL3));
  }

  public Command goToStation() {
    return arm
        .InstantPIDCommand(ArmConstants.kIntake)
        .andThen(elevator.InstantPIDCommand(ElevatorConstants.INTAKE));
  }

  public Command stow() {
    return elevator.InstantPIDCommand(ElevatorConstants.STOW)
        .alongWith(arm.InstantPIDCommand(ArmConstants.kSTOW));
  }

  public Command goToL1Auto() {
    return arm
        .InstantPIDCommand(ArmConstants.kL1)
        .andThen(new WaitCommand(0.5))
        .andThen(elevator.InstantPIDCommand(ElevatorConstants.L1));
  }

  public Command goToL2Auto() {
    return arm
        .InstantPIDCommand(ArmConstants.kL2)
        .andThen(new WaitCommand(0.5))
        .andThen(elevator.InstantPIDCommand(ElevatorConstants.L2));
  }

  public Command goToL3Auto() {
    return arm
        .InstantPIDCommand(ArmConstants.kL3)
        .andThen(new WaitCommand(0.5))
        .andThen(elevator.InstantPIDCommand(ElevatorConstants.L3));
  }

  public Command goToL4Auto() {
    return arm
        .InstantPIDCommand(ArmConstants.kL4)
        .andThen(new WaitCommand(0.5))
        .andThen(elevator.InstantPIDCommand(ElevatorConstants.L4));
  }

  public Command stowAuto() {
    return elevator.PIDCommand(ElevatorConstants.STOW)
        .alongWith(arm.PIDCommand(ArmConstants.kSTOW));
  }

  public Command goToStationAuto() {
    return elevator.InstantPIDCommand(ElevatorConstants.INTAKE)
        .alongWith(arm.InstantPIDCommand(ArmConstants.kIntake));
  }

  public Command coralIntake() {
    return m_gripper.Intake();
  }

  public Command coralIntakeForever() {
    return null;
  }

  public Command coralOuttakeForever() {
    return null;
  }

  public Command coralFeeder() {
    return arm
        .PIDCommand(ArmConstants.kIntake)
        .andThen(coralIntake());
  }

  public Command coralOuttake() {
    return m_gripper.outtake();
  }

  public void updateSimulation() {
    if (Constants.currentMode != Constants.Mode.SIM)
      return;

    SimulatedArena.getInstance().simulationPeriodic();
    Logger.recordOutput("FieldSimulation/RobotPosition", swerveDriveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
    Logger.recordOutput(
        "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
  }

  public void resetSimulation() {
    if (Constants.currentMode != Constants.Mode.SIM)
      return;

    // drive.resetOdometry(new Pose2d(3, 3, new Rotation2d()));
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  public Command fullL1() {
    return (goToL1().andThen(coralOuttake()));
  }

  public Command fullL2() {
    return (goToL2().andThen(coralOuttake()));
  }

  public Command fullL3() {
    return (goToL3().andThen(coralOuttake()));
  }

  public Command fullL1Auto() {
    return (goToL1Auto().andThen(coralOuttake()));
  }

  public Command fullL2Auto() {
    return (goToL2Auto().andThen(coralOuttake()));
  }

  public Command fullL3Auto() {
    return (goToL3Auto().andThen(coralOuttake()));
  }
}
