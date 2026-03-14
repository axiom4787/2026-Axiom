// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Conveyor;
import frc.robot.Constants.Indexer;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.Constants.IntakeArm;
import frc.robot.Constants.IntakeRoller;
import frc.robot.Constants.Swerve;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

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

  // SwerveInputStream driveAngularVelocity = SwerveInputStream.of(m_swerveSubsystem.getSwerveDrive(),
  //     () -> m_driverController.getLeftY() * -1,
  //     () -> m_driverController.getLeftX() * -1)
  //     .withControllerRotationAxis(m_driverController::getRightX)
  //     .deadband(Constants.Swerve.CONTROLLER_DEADBAND)
  //     .scaleTranslation(0.8)
  //     .allianceRelativeControl(false);

  public RobotContainer() {
    NEURALINK();
  }

  /** Optimal button binds to minimize latency from driver cortex to robot processor. */
  private void NEURALINK() {
    // Left Bumper: Toggle intake arm up/down.
    m_driverController.leftBumper().onTrue(new InstantCommand(() -> {
      boolean armDown = m_intakeArmSubsystem.isArmDown();
      m_intakeArmSubsystem.setArmPower(armDown ? IntakeArm.UP_POWER : IntakeArm.DOWN_POWER);
    }, m_intakeArmSubsystem));

    // Left Trigger: Run the intake rollers while held. Only runs if the intake arm is down.
    // Also runs the conveyor forward and the indexer backward to ensure all fuel remains within the hopper.
    m_driverController.leftTrigger(0.25).and(m_intakeArmSubsystem::isArmDown).whileTrue(new RunCommand(() -> {
      m_intakeRollerSubsystem.setRollerPower(IntakeRoller.INTAKE_POWER);
      m_conveyorSubsystem.setPower(Conveyor.FEED_POWER);
      m_indexerSubsystem.setPower(Indexer.EJECT_POWER);
    }, m_intakeRollerSubsystem, m_intakeArmSubsystem));

    // Right Bumper: Toggle shooter flywheel on/off.
    m_driverController.rightBumper().toggleOnTrue(new RunCommand(() -> {
      m_flywheelSubsystem.setDesiredSpeed(
        SmartDashboard.getNumber("Shooter/Setpoint", 0));
    }, m_flywheelSubsystem));

    // Right Trigger: Feed fuel into the shooter. Only runs if the shooter is at its desired speed.
    // The conveyor, intake, and indexer all run forward.
    m_driverController.rightTrigger(0.25).and(m_flywheelSubsystem::atSpeed).whileTrue(new RunCommand(() -> {
      m_intakeRollerSubsystem.setRollerPower(IntakeRoller.INTAKE_POWER);
      m_indexerSubsystem.setPower(Indexer.FEED_POWER);
      m_conveyorSubsystem.setPower(Conveyor.FEED_POWER);
    }, m_intakeRollerSubsystem, m_indexerSubsystem, m_conveyorSubsystem)); // TODO: Ensure aimed at target and pose locked
    
    // TODO: add outtake functionality
  }

  private void configureBindings() {
    // Intake binds: Right bumper toggles the intake between enabled (arm down, rollers on) and disabled (arm up, rollers off)
    // The robot always starts the match in the intake disabled state, so that the arm is within the frame perimeter
    // m_driverController.rightBumper().whileTrue(new RunCommand(() -> {
    //   m_intakeRollerSubsystem.setRollerPower(IntakeRoller.INTAKE_POWER);
    //   // m_intakeArmSubsystem.setArmPower(IntakeArm.DOWN_POWER);
    // }, m_intakeRollerSubsystem));
    
    // m_driverController.b().whileTrue(new RunCommand(() -> {
    //   m_intakeRollerSubsystem.setRollerPower(-1);
    // }, m_intakeRollerSubsystem));

    m_driverController.rightBumper().toggleOnTrue(new RunCommand(() -> {
      // m_flywheelSubsystem.setDesiredSpeed(-20);
      // m_indexerSubsystem.setPower(0.5);
      m_intakeRollerSubsystem.setRollerPower(IntakeRoller.INTAKE_POWER);
      // m_intakeArmSubsystem.setArmPower(IntakeArm.DOWN_POWER);
    }, m_intakeRollerSubsystem));

    m_intakeRollerSubsystem.setDefaultCommand(new RunCommand(() -> {
      m_intakeRollerSubsystem.setRollerPower(0);
      // m_intakeArmSubsystem.setArmPower(IntakeArm.UP_POWER);
    }, m_intakeRollerSubsystem));

    // Swerve Drive command
    // Command driveFieldOrientedAngularVelocityCommand = new RunCommand(() -> {m_swerveSubsystem.drive(driveAngularVelocity.get());}, m_swerveSubsystem);
    Command driveCommand = m_swerveSubsystem.driveCommand(
      () -> MathUtil.applyDeadband(m_driverController.getLeftY() * -1, Swerve.CONTROLLER_DEADBAND), 
      () -> MathUtil.applyDeadband(m_driverController.getLeftX() * -1, Swerve.CONTROLLER_DEADBAND), 
      () -> MathUtil.applyDeadband(m_driverController.getRightX() * 0.8, Swerve.CONTROLLER_DEADBAND));
    m_swerveSubsystem.setDefaultCommand(driveCommand);
  
   // Shooter binds: Left bumper toggles the shooter enabled/disabled. This just determines whether the flywheels are on or off
   // Fuel will not actually be fired until the indexer is enabled to feed fuel into the shooter
		m_driverController.leftBumper().toggleOnTrue(new RunCommand(() -> {
			m_flywheelSubsystem.setDesiredSpeed(
				SmartDashboard.getNumber("Shooter/Setpoint", 0));
		}, m_flywheelSubsystem));

		m_flywheelSubsystem.setDefaultCommand(new RunCommand(() -> {
			m_flywheelSubsystem.setDesiredSpeed(0.0);
		}, m_flywheelSubsystem));
  
    // Fuel will only be fed into the shooter if the flywheels are at their target speed and the Y button is held
    Trigger feed = m_driverController.y().and(m_flywheelSubsystem::atSpeed);

    // Both the indexer and conveyor should be enabled when feeding fuel to ensure the hopper is fully cleared out
    feed.whileTrue(new RunCommand(() -> {
      m_indexerSubsystem.setPower(Indexer.FEED_POWER); 
      // m_conveyorSubsystem.setPower(Conveyor.POWER);
    }, m_indexerSubsystem, m_conveyorSubsystem));

    // When not feeding, the indexer and conveyor should be disabled
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
