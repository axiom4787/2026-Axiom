package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.util.Units;


public class Constants {
    public static final class Swerve {
        public static final double MAX_SPEED = 4.7;
        public static final double CONTROLLER_DEADBAND = 0.1;
    }

    public static final class Hubs {
        public static final Pose2d RED_HUB = new Pose2d();
        public static final Pose2d BLUE_HUB = new Pose2d();
        // TODO: Locate the Hubs
    }

    public static class Vision {
        public static final String CAMERA_NAME = "Arducam_OV9281_USB_Camera";
        // Cam mounted facing forward, half a meter forward of center, half a meter up from center,
        // pitched upward.
        private static final double CAMERA_PITCH = Units.degreesToRadians(30.0);
        public static final Transform3d ROBOT_TO_CAM =
                new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, -CAMERA_PITCH, 0));

        public static final boolean UPDATE_HEADING_FROM_VISION = true;
        
        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout TAG_LAYOUT =
                AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        // The standard deviations of our vision estimated poses, which affect correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, 1);
    }
}
