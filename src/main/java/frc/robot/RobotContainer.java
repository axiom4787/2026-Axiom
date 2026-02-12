// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import org.dyn4j.geometry.Triangle;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Conveyor;
import frc.robot.Constants.Flywheel;
import frc.robot.Constants.Indexer;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.Constants.IntakeArm;
import frc.robot.Constants.IntakeRoller;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

public class RobotContainer {
  private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem((new File(Filesystem.getDeployDirectory(),
      "swerve/robot")));
  private final CommandXboxController m_driverController = new CommandXboxController(0);
  private final FlywheelSubsystem m_flywheelSubsystem = new FlywheelSubsystem();

  private final IntakeRollerSubsystem m_intakeRollerSubsystem = new IntakeRollerSubsystem();
  private final IntakeArmSubsystem m_intakeArmSubsystem = new IntakeArmSubsystem();
  
  private final ConveyorSubsystem m_conveyorSubsystem = new ConveyorSubsystem();
  private final IndexerSubsystem m_indexerSubsystem = new IndexerSubsystem();

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(m_swerveSubsystem.getSwerveDrive(),
      () -> m_driverController.getLeftY() * -1,
      () -> m_driverController.getLeftX() * -1)
      .withControllerRotationAxis(m_driverController::getRightX)
      .deadband(Constants.Swerve.CONTROLLER_DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    m_driverController.rightBumper().toggleOnTrue(new RunCommand(() -> {
      m_intakeRollerSubsystem.setRollerPower(IntakeRoller.INTAKE_POWER);
      m_intakeArmSubsystem.setArmPower(IntakeArm.DOWN_POWER);
    }, m_intakeRollerSubsystem));

    m_intakeRollerSubsystem.setDefaultCommand(new RunCommand(() -> {
      m_intakeRollerSubsystem.setRollerPower(0.0);
      m_intakeArmSubsystem.setArmPower(IntakeArm.UP_POWER);
    }, m_intakeRollerSubsystem));

    Command driveFieldOrientedAngularVelocityCommand = m_swerveSubsystem.driveFieldOriented(driveAngularVelocity);
    m_swerveSubsystem.setDefaultCommand(driveFieldOrientedAngularVelocityCommand);
  
   // shooter enabled/disabled logic
		m_driverController.leftBumper().toggleOnTrue(new RunCommand(() -> {
			m_flywheelSubsystem.setDesiredSpeed(
				SmartDashboard.getNumber("Shooter/Setpoint", 0));
		}, m_flywheelSubsystem));

		m_flywheelSubsystem.setDefaultCommand(new RunCommand(() -> {
			m_flywheelSubsystem.setDesiredSpeed(0.0);
		}, m_flywheelSubsystem));
  
    Trigger feed = m_driverController.y().and(m_flywheelSubsystem::atSpeed);

    feed.whileTrue(new RunCommand(() -> {
      m_indexerSubsystem.setPower(Indexer.POWER); 
      m_conveyorSubsystem.setPower(Conveyor.POWER);
    }, m_indexerSubsystem, m_conveyorSubsystem));

    m_conveyorSubsystem.setDefaultCommand(new RunCommand(() -> {
      m_conveyorSubsystem.setPower(0);
    }, m_conveyorSubsystem));

    m_indexerSubsystem.setDefaultCommand(new RunCommand(() -> {
      m_indexerSubsystem.setPower(0);
    }, m_indexerSubsystem));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
