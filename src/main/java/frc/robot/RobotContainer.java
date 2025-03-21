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

import static frc.robot.subsystems.elevator.ElevatorConstants.L1;
import static frc.robot.subsystems.elevator.ElevatorConstants.L2;
import static frc.robot.subsystems.elevator.ElevatorConstants.L3;
import static frc.robot.subsystems.elevator.ElevatorConstants.L4;
import static frc.robot.util.drive.DriveControls.*;
import frc.robot.commands.AprilTagAlign.AprilTagAlignLeft;
import frc.robot.commands.AprilTagAlign.AprilTagAlignMiddle;
import frc.robot.commands.AprilTagAlign.ApriltagAlignRight;

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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.PhotonVision.VisionIO;
import frc.robot.subsystems.PhotonVision.VisionIOPhoton;
import frc.robot.subsystems.PhotonVision.VisionIOPhotonSim;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.arm.ArmIOSparkMax;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOReal;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkMax;
import frc.robot.subsystems.elevator.ElevatorIOSparkFlex;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.gripper.GripperIO;
import frc.robot.subsystems.gripper.GripperIOSim;
import frc.robot.subsystems.gripper.GripperIOSparkMax;
import frc.robot.util.drive.DriveControls;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;
//~
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSIM;
import frc.robot.subsystems.hanger.*;

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

  private boolean brakeMode = true;

  private Mechanism2d elevatorMech = new Mechanism2d(3, 3);

  // LEDs

  // Dashboard inputs
  private LoggedDashboardChooser<Command> autoChooser;
  private LoggedDashboardBoolean brakeModeDashboard = new LoggedDashboardBoolean("Brake Mode", true);
  private LoggedDashboardBoolean setStartPosition = new LoggedDashboardBoolean("Set Start Position", false);

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
        // Sim robot, instantiate physics sim IO implementations
        elevator = new Elevator(new ElevatorIOSIM());
        drive = new Drive(
            new GyroIO() {
            },
            new ModuleIOSim(),
            new ModuleIOSim(),
            new ModuleIOSim(),
            new ModuleIOSim(),
            new VisionIOPhotonSim());
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
        new SequentialCommandGroup(elevator.PIDCommand(ElevatorConstants.L4).withTimeout(1.5),
            arm.PIDCommand(ArmConstants.kL4).withTimeout(0.5)));
    NamedCommands.registerCommand("IntakeSetpoint",
        new ParallelCommandGroup(elevator.PIDCommand(ElevatorConstants.INTAKE).withTimeout(2),
            new SequentialCommandGroup(new WaitCommand(0), arm.PIDCommand(ArmConstants.kIntake).withTimeout(1))));
    NamedCommands.registerCommand("Intake", m_gripper.Intake());
    NamedCommands.registerCommand("Outtake", new ParallelDeadlineGroup(m_gripper.outtake(),new SequentialCommandGroup(new WaitCommand(0.5), arm.PIDCommand(ArmConstants.kSTOW))));
    NamedCommands.registerCommand("AlignReefLeft", new ParallelCommandGroup(new AprilTagAlignLeft( drive, ()->0, ()->0), new WaitCommand(0)).andThen(DriveCommands.joystickDrive(drive, ()->0, ()->0, ()->0).withTimeout(0.001))
    );
    NamedCommands.registerCommand("AngleReset", new InstantCommand(() -> {drive.resetYaw();}));
    NamedCommands.registerCommand("Stow",
        new ParallelCommandGroup(elevator.PIDCommand(ElevatorConstants.STOW).withTimeout(1.5),
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
    DRIVE_PHOTONVISION_ALIGN_RIGHT
        .whileTrue(
            new SequentialCommandGroup(new AprilTagAlignLeft( drive, DRIVE_FORWARD, DRIVE_STRAFE),
                DriveCommands.joystickDriveRobotRelative(drive, () -> -0.4, () -> -0.05, () -> 0).withTimeout(.45))
                .withTimeout(20));
    DRIVE_PHOTONVISION_ALIGN_LEFT
        .whileTrue(
            new SequentialCommandGroup(new AprilTagAlignLeft( drive, DRIVE_FORWARD, DRIVE_STRAFE))
                .withTimeout(20));
    DRIVE_PHOTONVISION_ALIGN_MIDDLE
        .whileTrue(
            new SequentialCommandGroup(new AprilTagAlignMiddle(m_rotator.getHID(), drive, DRIVE_FORWARD, DRIVE_STRAFE),
            DriveCommands.joystickDriveRobotRelative(drive, () -> -0.4, () -> 0.05, () -> 0).withTimeout(.21))
                .withTimeout(20));
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

    MELTDOWN.onTrue(new SequentialCommandGroup(new InstantCommand(() -> {
      elevator.setVelocity(0);
    }, elevator), arm.stop(), m_gripper.stop(), new InstantCommand(() -> {
      hanger.stop();
    })));
    // Arm Commands
    arm.setDefaultCommand(arm.ManualCommand(PIVOT_ROTATE));

    // Gripper Commands
    OUTTAKE.onTrue(new ParallelCommandGroup(m_gripper.outtake(),new SequentialCommandGroup(new WaitCommand(0.15), arm.PIDCommand(ArmConstants.kSTOW)) ));
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
    DriveControls.STOW.onTrue(new ParallelCommandGroup(elevator.PIDCommand(ElevatorConstants.STOW),
        new SequentialCommandGroup(new WaitCommand(0), arm.PIDCommand(ArmConstants.kSTOW))));

    DriveControls.L1.onTrue(new ParallelCommandGroup(elevator.PIDCommand(ElevatorConstants.L1),
        new SequentialCommandGroup(new WaitCommand(1), arm.PIDCommand(ArmConstants.kL1))));

    DriveControls.L2.onTrue(new ParallelCommandGroup(elevator.PIDCommand(ElevatorConstants.L2),
        new SequentialCommandGroup(new WaitCommand(1), arm.PIDCommand(ArmConstants.kL2))));

    DriveControls.L3.onTrue(new ParallelCommandGroup(elevator.PIDCommand(ElevatorConstants.L3),
        new SequentialCommandGroup(new WaitCommand(1), arm.PIDCommand(ArmConstants.kL3))));

    DriveControls.L4.onTrue(new ParallelCommandGroup(elevator.PIDCommand(ElevatorConstants.L4),
        new SequentialCommandGroup(new WaitCommand(1), arm.PIDCommand(ArmConstants.kL4))));
    
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
    return autoChooser.get();
  }
}
