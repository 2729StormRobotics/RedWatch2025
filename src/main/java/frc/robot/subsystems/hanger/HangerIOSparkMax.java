package frc.robot.subsystems.hanger;

import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import frc.robot.subsystems.hanger.HangerConstants;;

public class HangerIOSparkMax implements HangerIO {

  private SparkMax hangerSparkMax;
  private SparkMaxConfig hangerConfig;
  public static boolean isClosed = false;
  public static Timer timer = new Timer();
  public SparkLimitSwitch metalDetector;

  public HangerIOSparkMax() {
    // Define motor
    hangerSparkMax = new SparkMax(HangerConstants.kWinchMotorCanId, MotorType.kBrushless);

    // Define Configs for Hanger Motor

    hangerConfig = new SparkMaxConfig();
    hangerConfig.idleMode(IdleMode.kBrake);

    // burn motor
    hangerSparkMax.configure(hangerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    metalDetector = hangerSparkMax.getForwardLimitSwitch();
  }

  @Override
  public void setHangerVoltage(double volts) {
    hangerSparkMax.setVoltage(volts);
  }

  @Override
  public double getHangerVoltage() {
    return hangerSparkMax.getBusVoltage() * hangerSparkMax.getAppliedOutput();
  }

  @Override
  public double getHangerAngle() {
    // NEEDS TO BE TESTED
    return 0;
  }

  @Override
  public boolean getIsInCage(){
    return metalDetector.isPressed();
  }

  @Override
  public void pull() {
    hangerSparkMax.set(HangerConstants.motorSpeedOpenHanger);
    isClosed = true;
  }

  @Override
  public void release() {
    hangerSparkMax.set(-HangerConstants.motorSpeedOpenHanger);
    isClosed = false;
  }

  @Override
  public void stop() {
    hangerSparkMax.set(0);
  }
  @Override
  public SequentialCommandGroup retract() {
    return new SequentialCommandGroup(
      new InstantCommand(() -> {System.out.println("retract");}),
        new InstantCommand(() -> {
          this.pull();
        }),
        new WaitCommand(5),
        new InstantCommand(() -> {
          this.stop();
        }));
  }
  @Override
  public SequentialCommandGroup extend() {
    return new SequentialCommandGroup(
        new InstantCommand(() -> {System.out.println("Extend");}),
        new InstantCommand(() -> {
          this.release();
        }),
        new WaitCommand(5),
        new InstantCommand(() -> {
          this.stop();
        }));
  }

  public void periodic(){
    SmartDashboard.putBoolean("metaldetector", getIsInCage());
  }

}
