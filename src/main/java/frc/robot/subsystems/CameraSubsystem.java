package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CameraSubsystem extends SubsystemBase {
    // private final MedianFilter filteredX = new MedianFilter(5);

    private final PhotonCamera camera;

    private boolean targetValid = false;
    private double targetX;
    private double targetY;

    public CameraSubsystem(String direction) {
        if (direction.equals("front")) {
            camera = new PhotonCamera(NetworkTableInstance.getDefault(), "Front Camera");
        } else if (direction.equals("rear")) {
            camera = new PhotonCamera(NetworkTableInstance.getDefault(), "Rear Camera");
        } else {
            camera = null;
        }

        camera.setPipelineIndex(DriverStation.getAlliance() == Alliance.Red ? 0 : 1);
    }

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
        SmartDashboard.putBoolean("Camera targetValid", targetValid);
    }

    public double getX() {
        return targetX;
    }

    public double getY() {
        return targetY;
    }

    public boolean getValid() {
        return targetValid;
    }

}