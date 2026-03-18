// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeRoller;

/** Represents the rollers which intake fuel into the robot's hopper. */
public class IntakeRollerSubsystem extends SubsystemBase {
  private final SparkFlex m_motor = new SparkFlex(
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
    config.smartCurrentLimit(40);
    config.encoder.velocityConversionFactor(IntakeRoller.INTAKE_CONVERSION_FACTOR);

    m_motor.configure(
        config,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    SmartDashboard.putNumber("Intake/VelocityFF", IntakeRoller.INTAKE_V);
    SmartDashboard.putNumber("Intake/DSPD", 0.0);
  }

  public void setDesiredSpeed() {
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
    m_desiredSpeed = SmartDashboard.getNumber("Intake/DSPD", 0.0);

    m_intakeFF.setKv(SmartDashboard.getNumber("Intake/VelocityFF", 0.0));
    SmartDashboard.putData(m_intakePID);

    double feedforward = m_intakeFF.calculate(m_desiredSpeed);
    double feedback = m_intakePID.calculate(m_currentSpeed, m_desiredSpeed);

    SmartDashboard.putNumber("Intake/Feedforward", feedforward);
    SmartDashboard.putNumber("Intake/Feedback", feedback);

    SmartDashboard.putNumber("Intake/Current Speed", m_currentSpeed);

    m_motor.setVoltage(feedforward + feedback);
  }
}
