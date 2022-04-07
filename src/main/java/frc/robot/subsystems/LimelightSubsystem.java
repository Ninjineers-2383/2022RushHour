package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
        } else {
            targetValid = false;
        }

        // post to smart dashboard
        SmartDashboard.putNumber("Target X", targetX);
        SmartDashboard.putNumber("Target Y", targetY);
    }

    public double getX() {
        return targetX;
    }

    public double getY() {
        return targetY;
    }

    public double getLaunchingVelocity() {
        // furthest for OG curve: -1.437
        // if (getY() < 5.96) {
        // return 14152 - 236 * x + 22 * x * x;
        // } else {
        // return -200 * x + 14900;
        // }
        // if (getY() > -1.437 && getY() < 16.5) {
        // return 14000 - 165 * getY();
        // } else {
        // return 15000 - 165 * getY();
        // }

        return -230 * getY() + 12050;
    }

    public boolean getTargetVisible() {
        return targetValid;
    }

    public void setLimelight(boolean isOn) {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        table.getEntry("ledMode").setNumber(isOn ? 0 : 2);
    }
}