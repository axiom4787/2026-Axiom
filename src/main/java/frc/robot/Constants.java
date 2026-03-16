package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.Matrix;


public class Constants {
    public static final class Swerve {
        public static final double MAX_SPEED = 4.7;
        public static final double CONTROLLER_DEADBAND = 0.1;
    }

    private static Pose2d makeTarget(double x, double y)
    {
        return new Pose2d(new Translation2d(x, y), Rotation2d.kZero);
    }

    public static final class Targets {
        public static final double FIELD_LENGTH = 16.5405;
        public static final double FIELD_WIDTH = 8.0695;

        public static final double BLUE_ALLIANCE_LINE_X = 4.4;
        public static final double RED_ALLIANCE_LINE_X = FIELD_LENGTH-4.4;
        public static final double CENTER_LINE_Y = FIELD_WIDTH/2;

        public static final Pose2d BLUE_HUB = makeTarget(4.625, CENTER_LINE_Y);
        public static final Pose2d RED_HUB = makeTarget(FIELD_LENGTH-4.625, CENTER_LINE_Y);
        public static final Pose2d BLUE_PASS_OUTPOST = makeTarget(1.5, 1.5);
        public static final Pose2d BLUE_PASS_DEPOT = makeTarget(1.5, FIELD_WIDTH-1.5);
        public static final Pose2d RED_PASS_OUTPOST = makeTarget(FIELD_LENGTH-1.5, FIELD_WIDTH-1.5);
        public static final Pose2d RED_PASS_DEPOT = makeTarget(FIELD_LENGTH-1.5, 1.5);
    }

    public static class Limelight {
        public static final String LL_NAME = "limelight-zarqa"; // Only owls will understand

        // Ignorance is bliss.
        public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, 1);
    }
}
