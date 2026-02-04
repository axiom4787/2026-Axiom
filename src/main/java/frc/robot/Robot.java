// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.Vision.*;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import swervelib.SwerveDrive;

import org.photonvision.PhotonCamera;


public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  private PhotonCamera camera;

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotInit() {
    camera = new PhotonCamera(kCameraName);
  }
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {

      // Read in relevant data from the Camera

      boolean targetVisible = false;

      double targetYaw = 0.0;

      var results = camera.getAllUnreadResults();

      if (!results.isEmpty()) {

          // Camera processed a new frame since last

          // Get the last one in the list.

          var result = results.get(results.size() - 1);

          if (result.hasTargets()) {

              // At least one AprilTag was seen by the camera

              for (var target : result.getTargets()) {

                  if (target.getFiducialId() == 0) {

                      // Found Tag 7, record its information

                      targetYaw = target.getYaw();

                      targetVisible = true;

                  }

              }

          }

      }
      
      SmartDashboard.putBoolean("Vision Target Visible", targetVisible);
      SmartDashboard.putNumber("Vision Target Yaw", targetYaw);

  }
  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
