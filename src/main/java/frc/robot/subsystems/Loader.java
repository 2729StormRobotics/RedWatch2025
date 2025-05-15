package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LoaderConstants;

public class Loader extends SubsystemBase {
  private final SparkMax loaderMotor;
  private final SparkMaxConfig loaderConfig;

  public Loader() {
    loaderMotor = new SparkMax(LoaderConstants.LOADER_MOTOR_ID, MotorType.kBrushless);
    loaderConfig = new SparkMaxConfig();
    configureMotor();
  }

  private void configureMotor() {
    SparkBaseConfig baseConfig = new SparkMaxConfig()
        .voltageCompensation(12.0)
        .smartCurrentLimit(30)
        .idleMode(LoaderConstants.LOADER_IDLE_MODE);

    loaderMotor.configure(baseConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void load(double speed) {
    // Apply speed limits
    speed = Math.max(-LoaderConstants.MAX_LOADER_SPEED, 
                    Math.min(LoaderConstants.MAX_LOADER_SPEED, speed));
    loaderMotor.set(speed);
  }

  public void stop() {
    loaderMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
} 