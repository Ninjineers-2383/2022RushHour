package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Limelight;
import frc.robot.Constants.Turret;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.TurretSubsystem;


public class LimelightCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  
  private final LimelightSubsystem limelight;

  private final DoubleSupplier turretTicks;

  private double turretPower = 0;

  private boolean turretSeek = false;

  private boolean kickerOn = false;

  private final double kP = 0.01;
  private DoubleSupplier drivetrainVelocity;

  MedianFilter drivetrainVelocityF = new MedianFilter(10);

  public LimelightCommand(LimelightSubsystem limelight, DoubleSupplier turretTicks, DoubleSupplier drivetrainVelocity) {
    this.limelight = limelight;
    this.turretTicks = turretTicks;
    this.drivetrainVelocity = drivetrainVelocity;
    
    addRequirements(limelight);
  }

  
  public void periodic() {
    SmartDashboard.putBoolean("Locked On", turretSeek);
    
  }
  

  //fix bounds issue!
  @Override
  public void execute() {
    kickerOn = false;
    turretSeek = false;
    
    double error = limelight.getX() + kP * drivetrainVelocity.getAsDouble() * Math.cos((turretTicks.getAsDouble() - 25000) * Math.PI / Turret.FULL_ROTATION);

    if(limelight.getTargetVisible()){
      limelight.setLimelight(true);
      turretPower = -Turret.kP * error;
    } else  {
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
