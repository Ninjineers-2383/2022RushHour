package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Turret;
import frc.robot.subsystems.LimelightSubsystem;

public class LimelightCommand extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private final LimelightSubsystem limelight;

    private double turretPower = 0;

    private boolean turretSeek = false;

    private boolean lockedOn = false;

    MedianFilter limelightF = new MedianFilter(5);

    DoubleSupplier driveVelocity;

    DoubleSupplier turretTicks;

    public LimelightCommand(LimelightSubsystem limelight) {
        this.limelight = limelight;
        this.driveVelocity = () -> 0;
        this.turretTicks = () -> 0;
        addRequirements(limelight);
    }

    public LimelightCommand(LimelightSubsystem limelight, DoubleSupplier turretTicks,
            DoubleSupplier drivetrainVelocity) {
        this.limelight = limelight;
        this.driveVelocity = drivetrainVelocity;
        this.turretTicks = turretTicks;
        addRequirements(limelight);
    }

    // fix bounds issue!
    @Override
    public void execute() {
        SmartDashboard.putBoolean("Locked On", lockedOn);

        turretSeek = false;

        double error = limelightF.calculate(limelight.getX());
        lockedOn = error < 0.5 && limelight.getTargetVisible();
        if (limelight.getTargetVisible()) {
            limelight.setLimelight(true);
            turretPower = -MathUtil.clamp(((Math.abs(error) > 0.4) ? 1 : 0) * (Turret.kP * error), -Turret.ADJUST_POWER,
                    Turret.ADJUST_POWER);
        } else {
            // no target present
            turretSeek = true;
        }
    }

    public double getTurretPower() {
        return turretPower;
    }

    public boolean getTurretSeek() {
        return turretSeek;
    }

    public boolean getLockedOn() {
        return lockedOn;
    }

    public boolean getTurretRunning() {
        return turretPower > 0.05;
    }

}
