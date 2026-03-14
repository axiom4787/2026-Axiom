package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.Matrix;


public class Constants {
    public static final class Swerve {
        public static final double MAX_SPEED = 4.7;
        public static final double CONTROLLER_DEADBAND = 0.1;
    }

    public static final class Hubs {
        public static final Pose2d RED_HUB = new Pose2d();
        public static final Pose2d BLUE_HUB = new Pose2d();
        // TODO: Triangulation.
    }

    public static class Limelight {
        public static final String LL_NAME = "Zarqa-Al-Yamama"; // Only owls will understand.

        // Ignorance is bliss.
        public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, 1);
    }
}
