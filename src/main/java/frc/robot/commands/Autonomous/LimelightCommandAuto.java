package frc.robot.commands.Autonomous;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Turret;
import frc.robot.subsystems.LimelightSubsystem;

public class LimelightCommandAuto extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

  private final LimelightSubsystem limelight;

  private double turretPower = 0;

  private boolean turretSeek = false;

  private boolean kickerOn = false;
  private int seenCycles = 0;

  MedianFilter drivetrainVelocityF = new MedianFilter(10);

  public LimelightCommandAuto(LimelightSubsystem limelight, DoubleSupplier turretTicks,
      DoubleSupplier drivetrainVelocity) {
    this.limelight = limelight;

    addRequirements(limelight);
  }

  public LimelightCommandAuto(LimelightSubsystem limelight, DoubleSupplier turretTicks,
      DoubleSupplier drivetrainVelocity, boolean velocityCompensation) {
    this.limelight = limelight;
    addRequirements(limelight);
  }

  public void periodic() {
    SmartDashboard.putBoolean("Locked On", turretSeek);

  }

  // fix bounds issue!
  @Override
  public void execute() {
    kickerOn = false;

    double error = limelight.getX();

    if (limelight.getTargetVisible()) {
      turretSeek = false;
      // limelight.setLimelight(true);
      SmartDashboard.putBoolean("isfine", true);
      turretPower = -Turret.kP * error;
      seenCycles++;
    } else {
      // no target present
      SmartDashboard.putBoolean("isfine", false);
      turretSeek = true;
      seenCycles = 0;
    }
  }

  @Override
  public boolean isFinished() {
    return seenCycles > 10;
  }

  @Override
  public void end(boolean end) {
    System.out.print("End");
    System.out.println(end);
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
