// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Conveyor;

/** Represents the conveyor that moves fuel through the robot's hopper into the shooter. */
public class ConveyorSubsystem extends SubsystemBase {
  private final SparkMax m_motor = new SparkMax(
      Conveyor.MOTOR_ID, MotorType.kBrushless);

  private double m_motorPower = 0.0;

  /** Creates a new ConveyorSubsystem. */
  public ConveyorSubsystem() {
    m_motor.setCANTimeout(250);

    SparkMaxConfig config = new SparkMaxConfig();
    config.inverted(false);
    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(15);

    m_motor.configure(
        config,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  /**
   * Sets the duty cycle of the conveyor motor, i.e. what percent of the time the motor is active.
   * @param motorPower A value between -1.0 and 1.0; negative values run the motor in reverse.
   */
  public void setPower(double motorPower) {
    m_motorPower = motorPower;
    m_motor.set(m_motorPower);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Conveyor/Motor Power", m_motorPower);
  }
}
