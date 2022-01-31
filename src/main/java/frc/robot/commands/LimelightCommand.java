package frc.robot.commands;

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
  

  @Override
  public void execute() {
    double sig = (Math.pow(Math.E, -0.15*(((limelight.getX())/2)-10)));
    kickerOn = false;
    turretSeek = false;
    if(limelight.getX() < -Limelight.LIMELIGHT_AIM_TOLERANCE) {
      // target on left
      turretPower = 0.25;
    } else if(limelight.getX() > Limelight.LIMELIGHT_AIM_TOLERANCE) {
      // target on right
      turretPower = -0.25;
    } else if(limelight.getTargetVisible()) {
      // target in center
        if((limelight.getX() < -1)) {
          turretPower = 1/(1 + sig);
        } else if((limelight.getX() > 1)) {
          turretPower = -1/(1 + sig);
        }
        else {
          turretPower = 0.0;
          kickerOn = true;
        }

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