package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Turret;
import frc.robot.subsystems.LimelightSubsystem;

public class LimelightCommand extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private final LimelightSubsystem limelight;

    private final MedianFilter limelightF = new MedianFilter(5);

    public LimelightCommand(LimelightSubsystem limelight) {
        this.limelight = limelight;
        addRequirements(limelight);
    }

    // fix bounds issue!
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
