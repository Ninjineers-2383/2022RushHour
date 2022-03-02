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

  private final DoubleSupplier turretTicks;

  private double turretPower = 0;

  private boolean turretSeek = false;

  private boolean kickerOn = false;

  private final boolean velocityCompensation;

  private final double kP = 0.00001;
  private DoubleSupplier drivetrainVelocity;

  private MedianFilter velocityFilter = new MedianFilter(5);

  MedianFilter drivetrainVelocityF = new MedianFilter(10);

  public LimelightCommand(LimelightSubsystem limelight, DoubleSupplier turretTicks, DoubleSupplier drivetrainVelocity) {
    this.limelight = limelight;
    this.turretTicks = turretTicks;
    this.drivetrainVelocity = drivetrainVelocity;
    this.velocityCompensation = false;
    
    addRequirements(limelight);
  }

  public LimelightCommand(LimelightSubsystem limelight, DoubleSupplier turretTicks, DoubleSupplier drivetrainVelocity, boolean velocityCompensation) {
    this.limelight = limelight;
    this.turretTicks = turretTicks;
    this.drivetrainVelocity = drivetrainVelocity;
    this.velocityCompensation = velocityCompensation;
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
    
    double error = limelight.getX() + (velocityCompensation ? 1:0) * kP * drivetrainVelocity.getAsDouble() * Math.cos((turretTicks.getAsDouble() - 25000) * Math.PI / Turret.FULL_ROTATION);

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
