// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeRoller;

/** Represents the rollers which intake fuel into the robot's hopper. */
public class IntakeRollerSubsystem extends SubsystemBase {
  private final SparkMax m_motor = new SparkMax(
      IntakeRoller.MOTOR_ID, MotorType.kBrushless);

  private final PIDController m_intakePID = new PIDController(
    IntakeRoller.INTAKE_P,
    IntakeRoller.INTAKE_I,
    IntakeRoller.INTAKE_D);

  private final SimpleMotorFeedforward m_intakeFF = new SimpleMotorFeedforward(
    IntakeRoller.INTAKE_S,
    IntakeRoller.INTAKE_V);

  private double m_currentSpeed = 0.0;
  private double m_desiredSpeed = 0.0;

  /** Creates a new IntakeRollerSubsystem. */
  public IntakeRollerSubsystem() {
    m_motor.setCANTimeout(250);

    SparkMaxConfig config = new SparkMaxConfig();
    config.inverted(false);
    config.idleMode(IdleMode.kCoast);
    config.smartCurrentLimit(60);
    config.encoder.velocityConversionFactor(IntakeRoller.INTAKE_CONVERSION_FACTOR);

    m_motor.configure(
        config,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public void setDesiredSpeed(double desiredSpeed) {
    m_desiredSpeed = desiredSpeed;
  }

  // /**
  //  * Sets the duty cycle of the intake roller motor, i.e. what percent of the time the motor is active.
  //  * @param motorPower A value between -1.0 and 1.0; negative values run the motor in reverse.
  //  */
  // public void setPower(double motorPower) {
  //   m_motorPower = motorPower;
  //   m_motor.set(m_motorPower);
  // }


  @Override
  public void periodic() {
    m_currentSpeed = m_motor.getEncoder().getVelocity();
    SmartDashboard.putNumber("Intake/Desired Speed", m_desiredSpeed);

    SmartDashboard.putData("Intake/Roller PID", m_intakePID);
    SmartDashboard.putNumber("Intake/Current Speed", m_currentSpeed);

    if (m_desiredSpeed == 0.0) {
      m_motor.setVoltage(0);
      SmartDashboard.putNumber("Intake/Feedforward", 0);
      SmartDashboard.putNumber("Intake/Feedback", 0);
      return;
    }

    double feedforward = m_intakeFF.calculate(m_desiredSpeed);
    double feedback = m_intakePID.calculate(m_currentSpeed, m_desiredSpeed);

    SmartDashboard.putNumber("Intake/Feedforward", feedforward);
    SmartDashboard.putNumber("Intake/Feedback", feedback);

    m_motor.setVoltage(feedforward + feedback);
  }
}
