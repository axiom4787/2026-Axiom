// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Conveyor;
import frc.robot.Constants.Indexer;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.Constants.IntakeRoller;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
  // Represents the main driver's controller
  private final CommandXboxController m_driverController = new CommandXboxController(0);

  // Initialize the robot's subsystems
  private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem((new File(Filesystem.getDeployDirectory(),
      "swerve/robot")));

  private final FlywheelSubsystem m_flywheelSubsystem = new FlywheelSubsystem();

  private final IntakeRollerSubsystem m_intakeRollerSubsystem = new IntakeRollerSubsystem();
  private final IntakeArmSubsystem m_intakeArmSubsystem = new IntakeArmSubsystem();
  
  private final ConveyorSubsystem m_conveyorSubsystem = new ConveyorSubsystem();
  private final IndexerSubsystem m_indexerSubsystem = new IndexerSubsystem();

  public RobotContainer() {
    NEURALINK();
  }

  /** Optimal button binds to minimize latency from driver cortex to robot processor. */
  private void NEURALINK() {
    // Left Bumper: Toggle between intake stowed and deployed.
    m_driverController.leftBumper().toggleOnTrue(new InstantCommand(() -> {
      m_intakeArmSubsystem.setDeployed(true);
    }, m_intakeArmSubsystem));

    // Left Trigger: Run the intake rollers while held. Only runs if the intake arm is deployed.
    // Also runs the conveyor forward and the indexer backward to ensure all fuel remains within the hopper.
    m_driverController.leftTrigger(0.25).and(m_intakeArmSubsystem::isDeployed).whileTrue(new RunCommand(() -> {
      m_intakeRollerSubsystem.setPower(IntakeRoller.INTAKE_POWER);
      m_conveyorSubsystem.setPower(Conveyor.FEED_POWER);
      m_indexerSubsystem.setPower(Indexer.EJECT_POWER);
    }, m_intakeRollerSubsystem, m_intakeArmSubsystem, m_conveyorSubsystem, m_indexerSubsystem));

    // Right Bumper: Toggle shooter flywheel on/off.
    m_driverController.rightBumper().toggleOnTrue(new RunCommand(() -> {
      m_flywheelSubsystem.setDesiredSpeed(
        SmartDashboard.getNumber("Shooter/Setpoint", 0));
    }, m_flywheelSubsystem));

    // Right Trigger: Feed fuel into the shooter. Only runs if the shooter is at its desired speed.
    // The conveyor, intake, and indexer all run forward.
    m_driverController.rightTrigger(0.25).and(m_flywheelSubsystem::atSpeed).whileTrue(new RunCommand(() -> {
      m_intakeRollerSubsystem.setPower(IntakeRoller.INTAKE_POWER);
      m_indexerSubsystem.setPower(Indexer.FEED_POWER);
      m_conveyorSubsystem.setPower(Conveyor.FEED_POWER);
    }, m_intakeRollerSubsystem, m_indexerSubsystem, m_conveyorSubsystem)); // TODO: Ensure aimed at target and pose locked
    
    // X: Eject all fuel from the robot if possible while held.
    // Conveyor, intake, indexer all run backward and the shooter is paused.
    // Additionally, the intake arm should deploy.
    m_driverController.x().whileTrue(new RunCommand(() -> {
      m_intakeArmSubsystem.setDeployed(true);
      m_intakeRollerSubsystem.setPower(IntakeRoller.OUTTAKE_POWER);
      m_indexerSubsystem.setPower(Indexer.EJECT_POWER);
      m_conveyorSubsystem.setPower(Conveyor.EJECT_POWER);
      m_flywheelSubsystem.setDesiredSpeed(0.0);
    }, m_intakeRollerSubsystem, m_intakeArmSubsystem, m_indexerSubsystem, m_conveyorSubsystem, m_flywheelSubsystem));

    // Default Binds: All subsystems disabled when not in use, intake stowed, shooter off, etc.
    
    m_intakeArmSubsystem.setDefaultCommand(new RunCommand(() -> {
      m_intakeArmSubsystem.setDeployed(false);
    }, m_intakeArmSubsystem));

    m_intakeRollerSubsystem.setDefaultCommand(new RunCommand(() -> {
      m_intakeRollerSubsystem.setPower(0);
    }, m_intakeRollerSubsystem));

    m_indexerSubsystem.setDefaultCommand(new RunCommand(() -> {
      m_indexerSubsystem.setPower(0);
    }, m_indexerSubsystem));

    m_conveyorSubsystem.setDefaultCommand(new RunCommand(() -> {
      m_conveyorSubsystem.setPower(0);
    }, m_conveyorSubsystem));

    m_flywheelSubsystem.setDefaultCommand(new RunCommand(() -> {
			m_flywheelSubsystem.setDesiredSpeed(0.0);
		}, m_flywheelSubsystem));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
