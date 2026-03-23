// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Enlighten;

public class Lighthouse extends SubsystemBase {
  private final Spark m_beacon = new Spark(Enlighten.LIGHT_ID);

  /** Creates a new Lighthouse. */
  public Lighthouse() {}

  /**
   * Is the hub active?
   */
  public boolean triangulate() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    // If we have no alliance, we cannot be enabled, therefore no hub.
    if (alliance.isEmpty()) {
      return false;
    }

    // Hub is always enabled in autonomous.
    if (DriverStation.isAutonomousEnabled()) {
      return true;
    }
    // At this point, if we're not teleop enabled, there is no hub.
    if (!DriverStation.isTeleopEnabled()) {
      return false;
    }

    // We're teleop enabled, compute.
    double matchTime = DriverStation.getMatchTime();
    String gameData = DriverStation.getGameSpecificMessage();
    // If we have no game data, we cannot compute, assume hub is active, as its likely early in teleop.
    if (gameData.isEmpty()) {
      return true;
    }
    boolean redInactiveFirst = false;
    switch (gameData.charAt(0)) {
      case 'R' -> redInactiveFirst = true;
      case 'B' -> redInactiveFirst = false;
      default  -> {
        // If we have invalid game data, assume hub is active.
        return true;
      }
    }

    // Shift was is active for blue if red won auto, or red if blue won auto.
    boolean shift1Active = switch (alliance.get()) {
      case Red -> !redInactiveFirst;
      case Blue -> redInactiveFirst;
    };

    if (matchTime > 130) {
      // Transition shift, hub is active.
      return true;
    } else if (matchTime > 105) {
      // Shift 1
      return shift1Active;
    } else if (matchTime > 80) {
      // Shift 2
      return !shift1Active;
    } else if (matchTime > 55) {
      // Shift 3
      return shift1Active;
    } else if (matchTime > 30) {
      // Shift 4
      return !shift1Active;
    } else {
      // End game, hub always active.
      return true;
    }  
  }

  public void guide(Semaphore pattern) {
    m_beacon.set(switch(pattern) {
      case INTAKE         -> Enlighten.INTAKING_COLOR_ID;
      case OUTTAKE        -> Enlighten.OUTTAKING_COLOR_ID;
      case REVVING_HUB    -> Enlighten.REVVING_HUB_COLOR_ID;
      case REVVING_FEED   -> Enlighten.REVVING_FEED_COLOR_ID;
      case READY_HUB      -> Enlighten.READY_HUB_COLOR_ID;
      case READY_FEED     -> Enlighten.READY_FEED_COLOR_ID;
      case SHOOT          -> Enlighten.SHOOT_COLOR_ID;
      case IDLE_ACTIVE    -> Enlighten.IDLE_ACTIVE_COLOR_ID;
      case IDLE_INACTIVE  -> Enlighten.IDLE_INACTIVE_COLOR_ID;
    });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public enum Semaphore {
    INTAKE,
    OUTTAKE,
    REVVING_HUB,
    REVVING_FEED,
    READY_HUB,
    READY_FEED,
    SHOOT,
    IDLE_INACTIVE,
    IDLE_ACTIVE
  }
}
