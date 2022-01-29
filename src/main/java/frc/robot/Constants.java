package frc.robot;

public final class Constants {
    //Hood angle: 90-16.1 degrees

    public final static double LIMELIGHT_AIM_TOLERANCE = 18; // 16 on each side, 32 total
    public final static int TURRET_INBOUNDS = 33000;
    public final static double SEEKING_POWER = 0.7;
    public final static double LIMELIGHT_ANGLE = 31; // measured in degrees
    public final static double LIMELIGHT_FOV = 45.7; // degrees
    public final static double LIMELIGHT_HEIGHT_DIFFERENCE = 103 - (42.25); // height difference between limelight camera and limelight tape

    public final class RobotMap {
        
        public final static int LAUNCHER_MASTER_PORT = 1;
        public final static int LAUNCHER_FOLLOWER_PORT = 2;

        public final static int TURRET_PORT = 3;

        public final static int RIGHT_MASTER_DRIVE_PORT = 4;
        public final static int LEFT_MASTER_DRIVE_PORT = 5;
        public final static int RIGHT_FOLLOWER_DRIVE_PORT = 6;
        public final static int LEFT_FOLLOWER_DRIVE_PORT = 7;

        public final static int KICKER_PORT = 8;

        public final static int CHIMNEY_PORT = 9;

        public final static int FRONT_FEEDER_PORT = 10;
        public final static int BACK_FEEDER_PORT = 11;
    }
}
