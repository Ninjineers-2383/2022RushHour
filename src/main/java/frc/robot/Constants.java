package frc.robot;

public final class Constants {
    public final static double LIMELIGHT_AIM_TOLERANCE = 22; // 16 on each side, 32 total
    public final static int TURRET_INBOUNDS = 33000;
    public final static double SEEKING_POWER = 0.7;
    public final static double LIMELIGHT_ANGLE = 33; //measured in degrees
    public final static double LIMELIGHT_FOV = 45.7;
    public final static double LIMELIGHT_HEIGHT_DIFFERENCE = 103 - (39.8125); // height difference between limelight camera and limelight tape

    public final class RobotMap {
        
        public final static int LAUNCHER_MASTER_PORT = 1;
        public final static int LAUNCHER_FOLLOWER_PORT = 2;

        public final static int TURRET_PORT = 3;
    }
}
