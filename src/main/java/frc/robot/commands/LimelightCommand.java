package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Turret;
import frc.robot.subsystems.LimelightSubsystem;

public class LimelightCommand extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    // Instance of the limelight
    private final LimelightSubsystem limelight;

    // Instance of the median filter to remove noise in the image
    private final MedianFilter limelightF;

    /**
     * A limelight command that takes in a limelight subsystem
     * 
     * @param limelight instance of limelight
     */
    public LimelightCommand(LimelightSubsystem limelight) {
        this.limelight = limelight;
        limelightF = new MedianFilter(5);
        addRequirements(limelight);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        limelight.setTurretSeek(false);

        double error = limelightF.calculate(limelight.getX());
        boolean lockedOn = error < 3 && limelight.getTargetVisible();
        limelight.setLockedOn(lockedOn);

        SmartDashboard.putBoolean("Locked On", lockedOn);

        if (limelight.getTargetVisible()) {
            limelight.setLimelight(true);
            double turretPower = -MathUtil.clamp(((Math.abs(error) > 0.4) ? 1 : 0) * (Turret.kP * error),
                    -Turret.ADJUST_POWER,
                    Turret.ADJUST_POWER);
            limelight.setTurretPower(turretPower);
        } else {
            // no target present
            limelight.setTurretSeek(true);
        }
    }
}
