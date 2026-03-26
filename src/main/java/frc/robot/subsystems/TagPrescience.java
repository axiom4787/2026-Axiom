// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.LimelightHelpers.PoseEstimate;

/** The state of knowing an effect before its cause has fully manifested. */
public class TagPrescience {
  /** A moment of clarity. */
  public record Revelation (boolean isManifest, PoseEstimate presence, Matrix<N3, N1> trust) {};

  private final String m_name;

  public TagPrescience(String name)
  {
    m_name = name;
  }

  /** Discerns a revelation. */
  public Revelation consult(double yaw) {
    // To remove pose ambiguity, the limelight needs to know the current robot orientation
    LimelightHelpers.SetRobotOrientation(m_name, yaw, 0, 0, 0, 0, 0);

    PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue(m_name);

    boolean isManifest = mt2.tagCount > 0;
    double baseStdDev = 0.05; // TODO: Tune

    int numTags = mt2.tagCount;
    double avgDist = mt2.avgTagDist;
    double xyStdDev = baseStdDev * Math.pow(avgDist, 2); // Far tags should be trusted much less than close tags

    xyStdDev /= Math.sqrt(numTags); // More tags should significiantly increase the trust of the measurement

    xyStdDev = Math.max(xyStdDev, 0.01); // We should never trust the vision too much

    Matrix<N3, N1> trust = VecBuilder.fill(xyStdDev, xyStdDev, 999999);
    return new Revelation(isManifest, mt2, trust);
  }
}
