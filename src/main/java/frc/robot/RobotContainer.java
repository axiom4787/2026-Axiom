// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Swerve;
import frc.robot.Constants.Targets;
import frc.robot.Constants.Conveyor;
import frc.robot.Constants.Indexer;
import frc.robot.Constants.IntakeArm;
import frc.robot.Constants.IntakeRoller;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.Lighthouse;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Lighthouse.Semaphore;
import frc.robot.subsystems.SwerveSubsystem.AimMode;
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

  // private final Lighthouse m_lighthouse = new Lighthouse();

  private final SendableChooser<Command> autoChooser;

  // Base input stream with no rotation control
  SwerveInputStream baseStream = SwerveInputStream.of(m_swerveSubsystem.getSwerveDrive(),
      () -> m_driverController.getLeftY() * -1,
      () -> m_driverController.getLeftX() * -1)
      .deadband(Swerve.CONTROLLER_DEADBAND)
      .allianceRelativeControl(true);

  // Input stream and command to drive robot with right joystick as rotation control
  SwerveInputStream driveAngularVelocity = baseStream.copy()
    .withControllerRotationAxis(() -> m_driverController.getRightX() * -1);
  Command driveAngVelCommand = m_swerveSubsystem.driveFieldOriented(driveAngularVelocity);
  
  // Same as previous, but with slowed inputs for precision
  Command driveAngVelSlowCommand = m_swerveSubsystem.driveFieldOriented(driveAngularVelocity.copy()
    .scaleTranslation(0.5)
    .scaleRotation(0.5));
  
  // Command to drive robot while aiming at a target
  Command driveWithAimCommand = m_swerveSubsystem.driveFieldOriented(baseStream.copy()
    .scaleTranslation(0.5)
    .aim(m_swerveSubsystem::getTarget)
    .aimWhile(true));

  public RobotContainer() {
    NEURALINK();
    registerNamedCommands();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /** Optimal button binds to minimize latency from driver cortex to robot processor. */
  private void NEURALINK() {
    // Left Bumper: Toggle between intake stowed and deployed.
    m_driverController.povUp().whileTrue(new RunCommand(() -> {
      m_intakeArmSubsystem.setPower(IntakeArm.STOW_SETPOINT);
    }, m_intakeArmSubsystem));

    m_driverController.povDown().whileTrue(new RunCommand(() -> {
      m_intakeArmSubsystem.setPower(IntakeArm.DEPLOY_SETPOINT);
    }, m_intakeArmSubsystem));

    // Left Trigger: Run the intake rollers while held. Only runs if the intake arm is deployed.
    // Also runs the conveyor forward and the indexer backward to ensure all fuel remains within the hopper.
    m_driverController.leftTrigger(0.25)/*.and(m_intakeArmSubsystem::isDeployed)*/.whileTrue(new RunCommand(() -> {
      m_intakeRollerSubsystem.setDesiredSpeed(IntakeRoller.INTAKE_SETPOINT);
      m_conveyorSubsystem.setPower(Conveyor.INTAKE_POWER);
      m_indexerSubsystem.setPower(Indexer.EJECT_POWER);
    }, m_intakeRollerSubsystem, m_conveyorSubsystem, m_indexerSubsystem));

    // m_driverController.leftTrigger(0.25)/*.and(m_intakeArmSubsystem::isDeployed)*/.whileTrue(new RunCommand(() -> {
    //   m_lighthouse.guide(Semaphore.INTAKE);
    // }, m_lighthouse));

    // Right Bumper: Toggle shooter flywheel on/off. Flywheel speed will be set based on the reported aim target.
    m_driverController.rightBumper().toggleOnTrue(new RunCommand(() -> {
      double dist = m_swerveSubsystem.getTargetDistance();
      if (m_swerveSubsystem.getAimMode() == AimMode.HUB) {
        m_flywheelSubsystem.setSpeedHubDist(dist);
      } else {
        m_flywheelSubsystem.setSpeedFeedDist(dist);
      }
    }, m_flywheelSubsystem));

    // m_driverController.rightBumper().toggleOnTrue(new RunCommand(() -> {
    //   if (m_swerveSubsystem.getAimMode() == AimMode.HUB) {
    //     if (m_flywheelSubsystem.atSpeed()) {
    //       m_lighthouse.guide(Semaphore.READY_HUB);
    //     } else {
    //       m_lighthouse.guide(Semaphore.REVVING_HUB);
    //     }
    //   } else {
    //     if (m_flywheelSubsystem.atSpeed()) {
    //       m_lighthouse.guide(Semaphore.READY_FEED);
    //     } else {
    //       m_lighthouse.guide(Semaphore.REVVING_FEED);
    //     }
    //   }
    // }, m_lighthouse));

    // Right Trigger: Feed fuel into the shooter. Only runs if the shooter is at its desired speed.
    // The conveyor, intake, and indexer all run forward.
    m_driverController.rightTrigger(0.25).and(m_flywheelSubsystem::atSpeed).whileTrue(new RunCommand(() -> {
      m_indexerSubsystem.setPower(Indexer.FEED_POWER);
      m_conveyorSubsystem.setPower(Conveyor.FEED_POWER);
    }, m_indexerSubsystem, m_conveyorSubsystem));

    // m_driverController.rightTrigger(0.25).and(m_flywheelSubsystem::atSpeed).whileTrue(new RunCommand(() -> {
    //   m_lighthouse.guide(Semaphore.SHOOT);
    // }, m_lighthouse).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    
    // X: Eject all fuel from the robot if possible while held.
    // Conveyor, intake, indexer all run backward and the shooter is paused.
    // Additionally, the intake arm should deploy.
    m_driverController.x().whileTrue(new RunCommand(() -> {
      m_intakeArmSubsystem.setPower(IntakeArm.DEPLOY_SETPOINT);
      // m_intakeRollerSubsystem.setPower(IntakeRoller.OUTTAKE_POWER);
      m_indexerSubsystem.setPower(Indexer.EJECT_POWER);
      m_conveyorSubsystem.setPower(Conveyor.EJECT_POWER);
      m_flywheelSubsystem.setDesiredSpeed(0.0);
    }, m_intakeRollerSubsystem, m_intakeArmSubsystem, m_indexerSubsystem, m_conveyorSubsystem, m_flywheelSubsystem));

    // m_driverController.x().whileTrue(new RunCommand(() -> {
    //   m_lighthouse.guide(Semaphore.OUTTAKE);
    // }, m_lighthouse));

    // Default Binds: All subsystems disabled when not in use, intake stowed, shooter off, etc.
    
    m_intakeArmSubsystem.setDefaultCommand(new RunCommand(() -> {
      m_intakeArmSubsystem.setPower(0.0);
    }, m_intakeArmSubsystem));

    m_intakeRollerSubsystem.setDefaultCommand(new RunCommand(() -> {
      m_intakeRollerSubsystem.setDesiredSpeed(0.0);
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

    // Default command for LED subsystem
    // m_lighthouse.setDefaultCommand(new RunCommand(() -> {
    //   if (m_lighthouse.triangulate()) {
    //     m_lighthouse.guide(Semaphore.IDLE_ACTIVE);
    //   } else {
    //     m_lighthouse.guide(Semaphore.IDLE_INACTIVE);
    //   }
		// }, m_lighthouse));

    // Drive controls
    // B: Aim at selected target while held.
    // Y: Drive slowly while held.
    // Left Stick: Lock wheels while held. Also triggers when shooting to counter defense.
    // A: Reset gyro heading
    
    m_driverController.b().whileTrue(new RunCommand(() -> {
      m_swerveSubsystem.driveFieldOriented(baseStream.copy()
        .scaleTranslation(0.5)
        .aim(m_swerveSubsystem::getTarget)
        .aimWhile(true).get());
    }, m_swerveSubsystem));
    m_driverController.y().whileTrue(driveAngVelSlowCommand);
    m_driverController.leftStick().whileTrue(new RunCommand(m_swerveSubsystem::lock, m_swerveSubsystem).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    m_driverController.rightTrigger().whileTrue(new RunCommand(m_swerveSubsystem::lock, m_swerveSubsystem)); // Separate from the previous command because we want to be able to continue moving while shooting if necessary
    m_driverController.a().onTrue(new InstantCommand(m_swerveSubsystem::zeroGyro, m_swerveSubsystem));

    m_swerveSubsystem.setDefaultCommand(driveAngVelCommand);
  }

  public void registerNamedCommands() {
    NamedCommands.registerCommand("Deploy Intake", new RunCommand(() -> m_intakeArmSubsystem.setPower(IntakeArm.DEPLOY_SETPOINT), m_intakeArmSubsystem)
      .withTimeout(1.5)
      .andThen(new InstantCommand(() -> m_intakeArmSubsystem.setPower(0.0), m_intakeArmSubsystem)));
    NamedCommands.registerCommand("Run Intake", new RunCommand(() -> m_intakeRollerSubsystem.setDesiredSpeed(IntakeRoller.INTAKE_SETPOINT), m_intakeRollerSubsystem)
      .withTimeout(3)
      .andThen(new InstantCommand(() -> m_intakeRollerSubsystem.setDesiredSpeed(0.0), m_intakeRollerSubsystem)));
    NamedCommands.registerCommand("Score Hub", new RunCommand(() -> m_flywheelSubsystem.setSpeedHubDist(m_swerveSubsystem.getTargetDistance()), m_flywheelSubsystem)
      .until(m_flywheelSubsystem::atSpeed)
      .andThen(new RunCommand(() -> {
        m_flywheelSubsystem.setSpeedHubDist(m_swerveSubsystem.getTargetDistance());
        m_conveyorSubsystem.setPower(Conveyor.FEED_POWER);
        m_indexerSubsystem.setPower(Indexer.FEED_POWER);
      }, m_conveyorSubsystem, m_indexerSubsystem, m_flywheelSubsystem).withTimeout(7.0))
      .andThen(new InstantCommand(() -> {
        m_indexerSubsystem.setPower(0);
        m_conveyorSubsystem.setPower(0);
        m_flywheelSubsystem.setDesiredSpeed(0);
      }, m_conveyorSubsystem, m_indexerSubsystem, m_flywheelSubsystem)));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
