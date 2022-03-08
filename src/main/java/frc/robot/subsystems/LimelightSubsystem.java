package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Limelight;


public class LimelightSubsystem extends SubsystemBase {
    // private final MedianFilter filteredX = new MedianFilter(5);

    private final PhotonCamera camera = new PhotonCamera(NetworkTableInstance.getDefault(), "limelight");

    private boolean targetValid = false;
    private double targetX;
    private double targetY;


    @Override
    public void periodic() {
        PhotonPipelineResult res = camera.getLatestResult();
        List<PhotonTrackedTarget> targets = res.targets;
        if (!targets.isEmpty()) {
            targetValid = true;
            PhotonTrackedTarget bestTarget = res.getBestTarget();
            targetX = bestTarget.getYaw();
            targetY = bestTarget.getPitch();
        }
        else {
            targetValid = false;
        }

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
        if (getY() > -1.437 && getY() < 16.5) {
            return 13500 - 165 * getY();
        } else {
            return 14500 - 165 * getY();
        }
    }

    public boolean getTargetVisible() {
        return targetValid;
    }
    public void setLimelight(boolean isOn) {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        table.getEntry("ledMode").setNumber(isOn ? 0:2);
    }
}