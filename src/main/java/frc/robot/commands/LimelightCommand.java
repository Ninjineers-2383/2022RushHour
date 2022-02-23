package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants.Limelight;
import frc.robot.subsystems.LimelightSubsystem;


public class LimelightCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  
  private final LimelightSubsystem limelight;

  private double turretPower = 0;

  private boolean turretSeek = false;

  private boolean kickerOn = false;
  

  public LimelightCommand(LimelightSubsystem limelight) {
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
    if(limelight.getX() < -Limelight.LIMELIGHT_AIM_TOLERANCE) {
      // target on left
      turretPower = -0.3;
    } else if(limelight.getX() > Limelight.LIMELIGHT_AIM_TOLERANCE) {
      // target on right
      turretPower = 0.3;
    } else if(limelight.getTargetVisible()) {
      // target in center
        if((limelight.getX() < -1.5)) {
          turretPower = 0.1;
        } else if((limelight.getX() > 1.5)) {
          turretPower = -0.1;
        }
        else {
          turretPower = 0.0;
          kickerOn = true;
        }
        //TODO: delete the code below
        // turretPower = 0.0;
        // kickerOn = true;

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
