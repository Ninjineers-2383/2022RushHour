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

    private boolean kickerOn = false;

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

    public void periodic() {
        SmartDashboard.putBoolean("Locked On", turretSeek);

    }

    // fix bounds issue!
    @Override
    public void execute() {
        kickerOn = false;
        turretSeek = false;

        final double CompensationAuthority = 0.0035;

        double error = MathUtil
                .clamp(limelightF.calculate(limelight.getX()) - CompensationAuthority * driveVelocity.getAsDouble()
                        * Math.signum(turretTicks.getAsDouble() - Turret.OFFSET_TICKS), -25, 25);
        SmartDashboard.putNumber("Compensation", error);
        if (limelight.getTargetVisible()) {
            limelight.setLimelight(true);
            turretPower = -MathUtil.clamp(((Math.abs(error) > 1) ? 1 : 0) * (Turret.kP * error), -Turret.SEEKING_POWER,
                    Turret.SEEKING_POWER);
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

    public boolean getKickerOn() {
        return kickerOn;
    }

}
