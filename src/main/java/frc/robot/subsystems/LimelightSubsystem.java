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
    private boolean lockedOn = false;
    private double turretPower = 0.0;
    private boolean turretSeek = false;

    public LimelightSubsystem() {
        camera.setPipelineIndex(0); // Set the pipeline to 0, make sure the comp pipeline is the first pipeline
    }

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
        double y = getY();
        // return -230 * y + 12500;
        return 12801 + (-86.1 * y) + (8.22 * y * y); // https://www.desmos.com/calculator/3hmh5agnm1
    }

    public boolean getTargetVisible() {
        return targetValid;
    }

    public void setLimelight(boolean isOn) {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        table.getEntry("ledMode").setNumber(isOn ? 0 : 2);
    }

    public void setLockedOn(boolean lockedOn) {
        this.lockedOn = lockedOn;
    }

    public boolean getLockedOn() {
        return lockedOn;
    }

    public void setTurretPower(double turretPower) {
        this.turretPower = turretPower;
    }

    public double getTurretPower() {
        return turretPower;
    }

    public void setTurretSeek(boolean turretSeek) {
        this.turretSeek = turretSeek;
    }

    public boolean getTurretSeek() {
        return turretSeek;
    }
}