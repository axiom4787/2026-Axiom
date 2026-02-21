// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Flywheel;

/** Represents the flywheels that fire fuel out of the robot. */
public class FlywheelSubsystem extends SubsystemBase {
  // Solo: The singular NEO Vortex motor on the left side
  // Duo Upper/Lower: The two NEO Vortex motors on the right side
  private final SparkFlex m_flywheelSoloMotor = new SparkFlex(Flywheel.FLYWHEEL_SOLO_MOTOR_ID, MotorType.kBrushless);
  private final SparkFlex m_flywheelDuoUpperMotor = new SparkFlex(Flywheel.FLYWHEEL_DUO_UPPER_MOTOR_ID,
      MotorType.kBrushless);
  // private final SparkFlex m_flywheelDuoLowerMotor = new SparkFlex(Flywheel.FLYWHEEL_DUO_LOWER_MOTOR_ID,
  //     MotorType.kBrushless);

  private final PIDController m_flywheelPID = new PIDController(
      Flywheel.FLYWHEEL_P,
      Flywheel.FLYWHEEL_I,
      Flywheel.FLYWHEEL_D);

  private final SimpleMotorFeedforward m_flywheelFF = new SimpleMotorFeedforward(
      Flywheel.FLYWHEEL_S,
      Flywheel.FLYWHEEL_V);

  private double m_desiredSpeed = 0;

  private double m_currentSpeed = 0;

  /** Creates a new FlywheelSubsystem. */
  public FlywheelSubsystem() {
    SparkMaxConfig config = new SparkMaxConfig();

    config
        .inverted(true)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(40).encoder
        .velocityConversionFactor(Flywheel.FLYWHEEL_CONVERSION_FACTOR);

    m_flywheelSoloMotor.configure(
        config,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    SparkMaxConfig config2 = new SparkMaxConfig();

    config2
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(40)
        .follow(m_flywheelSoloMotor, true).encoder.velocityConversionFactor(Flywheel.FLYWHEEL_CONVERSION_FACTOR);

    m_flywheelDuoUpperMotor.configure(
        config2,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // m_flywheelDuoLowerMotor.configure(
    //     config2,
    //     ResetMode.kResetSafeParameters,
    //     PersistMode.kPersistParameters);

    SmartDashboard.putNumber("Shooter/Setpoint", 0.0);
    SmartDashboard.putNumber("Shooter/Velocity Gain Setpoint", Flywheel.FLYWHEEL_V);
  }

  /**
   * Sets the desired speed (setpoint) of the flywheels. 
   * A combination of feedforward and PID control will bring the flywheels to the desired speed over time.
   * @param desiredSpeed The speed setpoint, in radians per second.
   */
  public void setDesiredSpeed(double desiredSpeed) {
    m_desiredSpeed = desiredSpeed;
  }

  /**
   * Whether or not the flywheels are at the correct speed (i.e. ready to fire a shot)
   * @return True if the current speed is within 0.5 radians per second of the speed setpoint, false otherwise.
   */
  public boolean atSpeed() {
    return Math.abs(m_desiredSpeed - m_currentSpeed) < 0.5;
  }

  @Override
  public void periodic() {
    m_flywheelFF.setKv(SmartDashboard.getNumber("Shooter/Velocity Gain Setpoint", Flywheel.FLYWHEEL_V));
    m_currentSpeed = m_flywheelSoloMotor.getEncoder().getVelocity();

    SmartDashboard.putNumber("Shooter/Desired Speed", m_desiredSpeed);
    SmartDashboard.putNumber("Shooter/Current Speed", m_currentSpeed);
    SmartDashboard.putData("Shooter/PID", m_flywheelPID);

    double feedforward = m_flywheelFF.calculate(m_desiredSpeed);

    double feedback = m_flywheelPID.calculate(m_currentSpeed, m_desiredSpeed);

    SmartDashboard.putNumber("Shooter/Feedforward", feedforward);
    SmartDashboard.putNumber("Shooter/Feedback", feedback);

    m_flywheelSoloMotor.setVoltage(feedback + feedforward);
  }
}
