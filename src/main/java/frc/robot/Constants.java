package frc.robot;


public final class Constants {

    public final static class Limelight {
        public final static double LIMELIGHT_AIM_TOLERANCE = 18;
    }

    //Hood angle: 90-16.1 degrees
    public final static class Turret {
        public final static int PORT = 3;
        public final static int BOUNDS = 33000;
        public final static double SEEKING_POWER = 0.7;
    } 


    public final static class Drivetrain {
        public final static int RIGHT_MASTER_PORT = 4;
        public final static int LEFT_MASTER_PORT = 5;
        public final static int RIGHT_FOLLOWER_PORT = 6;
        public final static int LEFT_FOLLOWER_PORT = 7;
    }


    public final static class Launcher {
        public final static int MASTER_PORT = 1;
        public final static int FOLLOWER_PORT = 2;
    }

    public final static class Intake {
        //solenoid ports on pcm
        public final static int FRONT_SOLENOID_PORT = 1;
        public final static int REAR_SOLENOID_PORT = 2;
        public final static int FRONT_INTAKE_PORT = 10;
        public final static int REAR_INTAKE_PORT = 11;
    }


    public final static class Kicker {
        public final static int PORT = 8;
    }
    

    public final static class Chimney {
        public final static int PORT = 9;
    }
}
