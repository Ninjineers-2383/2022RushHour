package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Limelight;


public class LimelightSubsystem extends SubsystemBase {
    private final MedianFilter filteredX = new MedianFilter(5);
    private final MedianFilter filteredY = new MedianFilter(1000);

    private boolean targetValid = false;
    private double targetX;
    private double targetY;


    @Override
    public void periodic() {
        // get limelight values from ethernet
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry tableValidTarget = table.getEntry("tv");
        NetworkTableEntry tableTargetX = table.getEntry("tx");
        NetworkTableEntry tableTargetY = table.getEntry("ty");

        // reads values
        targetValid = tableValidTarget.getDouble(1) != 0;
        targetX = filteredX.calculate(tableTargetX.getDouble(0));
//        targetY = filteredY.calculate(tableTargetY.getDouble(0));
        targetY = tableTargetY.getDouble(0);

        // post to smart dashboard
        SmartDashboard.putNumber("Target X", targetX);
        SmartDashboard.putNumber("Target Y", targetY);
        SmartDashboard.putBoolean("Ready To Fire", Math.abs(getX()) < Limelight.LIMELIGHT_AIM_TOLERANCE && targetValid);
    }


    public double getX() {
        return targetX;
    }

    public double getY() {
        return targetY;
    }

    public double getLaunchingVelocity() {
        //furthest for OG curve: -1.437
        if (getY() > -1.437) {
            return 13500 - 165 * getY();
        } else {
            return 14500 - 165 * getY();
        }
    }

    public boolean getTargetVisible() {
        return targetValid;
    }
}