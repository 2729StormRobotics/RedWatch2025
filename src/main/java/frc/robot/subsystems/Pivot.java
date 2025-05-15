package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;

public class Pivot extends SubsystemBase {
  private final SparkMax pivotMotor;
  private final SparkMaxConfig pivotConfig;

  public Pivot() {
    pivotMotor = new SparkMax(PivotConstants.PIVOT_MOTOR_ID, MotorType.kBrushless);
    pivotConfig = new SparkMaxConfig();
    configureMotor();
  }

  private void configureMotor() {
    SparkBaseConfig baseConfig = new SparkMaxConfig()
        .voltageCompensation(12.0)
        .smartCurrentLimit(30)
        .idleMode(PivotConstants.PIVOT_IDLE_MODE);

    pivotMotor.configure(baseConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void pivot(double speed) {
    // Apply speed limits
    speed = Math.max(-PivotConstants.MAX_PIVOT_SPEED, 
                    Math.min(PivotConstants.MAX_PIVOT_SPEED, speed));
    pivotMotor.set(speed);
  }

  public void moveToPosition(double targetPosition) {
    double currentPosition = pivotMotor.getAbsoluteEncoder().getPosition();
    double error = targetPosition - currentPosition;
    
    // Simple proportional control
    double speed = error * 0.01; // Adjust this constant as needed
    speed = Math.max(-PivotConstants.MAX_PIVOT_SPEED, 
                    Math.min(PivotConstants.MAX_PIVOT_SPEED, speed));
    pivotMotor.set(speed);
  }

  public void moveToIntakePosition() {
    moveToPosition(PivotConstants.INTAKE_POSITION);
  }

  public void moveToZeroPosition() {
    moveToPosition(PivotConstants.ZERO_POSITION);
  }

  public void stop() {
    pivotMotor.set(0);
  }

  public double getPosition() {
    return pivotMotor.getAbsoluteEncoder().getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
} 