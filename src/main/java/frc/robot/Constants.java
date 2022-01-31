package frc.robot;


public final class Constants {

    public final static class Limelight {
        public final static double LIMELIGHT_AIM_TOLERANCE = 18;
    }

    //Hood angle: 90-16.1 degrees
    public final static class Turret {
        public final static int TURRET_INBOUNDS = 33000;
        public final static double SEEKING_POWER = 0.7;
    } 
    
    
    public final static class RobotMap {

        //solenoid ports on pcm
        public final static int SOLENOID1_PORT = 1;
        public final static int SOLENOID2_PORT = 2;
        
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
