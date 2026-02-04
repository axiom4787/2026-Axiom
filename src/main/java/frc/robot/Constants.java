package frc.robot;

public class Constants {
    public static final class Swerve {
        public static final double MAX_SPEED = 4.7;
        public static final double CONTROLLER_DEADBAND = 0.1;
    }

    public static final class Flywheel {
        public static final int FLYWHEEL_MOTOR_ID = 10;
        
        //TODO: tune pid values
        public static final double FLYWHEEL_S = 0;
        public static final double FLYWHEEL_V = 0;
        public static final double FLYWHEEL_P = 0;
        public static final double FLYWHEEL_I = 0;
        public static final double FLYWHEEL_D = 0;

        public static final double FLYWHEEL_CONVERSION_FACTOR = (2.0 / Math.PI) / 60;
    }
}
