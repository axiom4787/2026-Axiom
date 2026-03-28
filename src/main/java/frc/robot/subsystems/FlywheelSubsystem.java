// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Flywheel;

/** Represents the flywheels that launch fuel out of the robot. */
public class FlywheelSubsystem extends SubsystemBase {
  private final SparkFlex m_rightMotor = new SparkFlex(Flywheel.RIGHT_MOTOR_ID, MotorType.kBrushless);
  private final SparkFlex m_leftMotor = new SparkFlex(Flywheel.LEFT_MOTOR_ID,
      MotorType.kBrushless);

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
    SparkFlexConfig config = new SparkFlexConfig();

    config
        .inverted(false)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(80).encoder
        .velocityConversionFactor(Flywheel.FLYWHEEL_CONVERSION_FACTOR);

    m_rightMotor.configure(
        config,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    SparkFlexConfig config2 = new SparkFlexConfig();

    config2
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(80)
        .follow(m_rightMotor, true).encoder.velocityConversionFactor(Flywheel.FLYWHEEL_CONVERSION_FACTOR);

    m_leftMotor.configure(
        config2,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
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
   * Sets the desired speed of the flywheel based on a regression for hub distance.
   * Targets the top of the hub (72 in. above floor)
   * @param hubDist Distance from the robot to the hub, in meters.
   */
  public void setSpeedHubDist(double hubDist) {
    double hubDistIn = Units.metersToInches(hubDist); // regression was tuned in inches measured from front of robot
    // m_desiredSpeed = 0.1*hubDist + 22.25; // high height, including regression data at 45 rad/s and above
    // m_desiredSpeed = 0.1185*hubDist + 18.635; // with lower hub target height (65 in.)
    m_desiredSpeed = 3.777704*Math.pow(hubDistIn, 0.455459); // reg height power
    // m_desiredSpeed = 3.4475*Math.pow(hubDist, 0.471726); // low height power
    // m_desiredSpeed = 0.11925*hubDist + 18.3; // with lowest hub target height (60 in.)
    // m_desiredSpeed = 0.1175*hubDist + 19.15; // ignoring regression data at 45 rad/s and above
  }

  /**
   * Sets the desired speed of the flywheel based on a regression for distance from the alliance zone.
   * Targets the closest corner of the alliance zone, about 2.15 meters away from the wall diagonally.
   * @param feedDistIn Distance from the robot to the alliance zone corner, in meters.
   */
  public void setSpeedFeedDist(double feedDist) {
    double feedDistIn = Units.metersToInches(feedDist)-7; // regression was tuned in inches measured from front of robot
    m_desiredSpeed = 1.542*Math.pow(feedDistIn, 0.62); // including full regression data, power regression
    // m_desiredSpeed = 0.1215*feedDistIn + 15.31;
  }

  /**
   * Whether or not the flywheels are at the correct speed (i.e. ready to fire a shot)
   * @return True if the current speed is within 0.5 radians per second of the speed setpoint, false otherwise.
   */
  public boolean atSpeed() {
    return Math.abs(m_desiredSpeed - m_currentSpeed) < 0.25;
  }

  @Override
  public void periodic() {
    m_currentSpeed = m_rightMotor.getEncoder().getVelocity();

    SmartDashboard.putString("Shooter/Velocity Text", String.format("%.2f", m_currentSpeed));
    SmartDashboard.putString("Shooter/Setpoint Text", String.format("%.2f", m_desiredSpeed));

    SmartDashboard.putNumber("Shooter/Velocity", m_currentSpeed);
    SmartDashboard.putNumber("Shooter/Setpoint", m_desiredSpeed);

    SmartDashboard.putData("Shooter/PID", m_flywheelPID);

    if (m_desiredSpeed == 0.0) {
      m_rightMotor.setVoltage(0);
      SmartDashboard.putNumber("Shooter/Feedforward", 0);
      SmartDashboard.putNumber("Shooter/Feedback", 0);
      return;
    }
    double feedforward = m_flywheelFF.calculate(m_desiredSpeed);

    double feedback = m_flywheelPID.calculate(m_currentSpeed, m_desiredSpeed);

    SmartDashboard.putNumber("Shooter/Feedforward", feedforward);
    SmartDashboard.putNumber("Shooter/Feedback", feedback);

    m_rightMotor.setVoltage(feedback + feedforward);
  }
}
