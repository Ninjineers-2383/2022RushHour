package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Turret;
import frc.robot.subsystems.LimelightSubsystem;


public class LimelightCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  
  private final LimelightSubsystem limelight;

  private double turretPower = 0;

  private boolean turretSeek = false;

  private boolean kickerOn = false;

  MedianFilter drivetrainVelocityF = new MedianFilter(10);

  public LimelightCommand(LimelightSubsystem limelight, DoubleSupplier turretTicks, DoubleSupplier drivetrainVelocity) {
    this.limelight = limelight;
    
    addRequirements(limelight);
  }

  public LimelightCommand(LimelightSubsystem limelight, DoubleSupplier turretTicks, DoubleSupplier drivetrainVelocity, boolean velocityCompensation) {
    this.limelight = limelight;
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
    
    double error = limelight.getX();

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
