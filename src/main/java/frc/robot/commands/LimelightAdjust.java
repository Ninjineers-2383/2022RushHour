package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class LimelightAdjust extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final LimelightSubsystem limelight;

  private double turretPower = 0;
  
  public LimelightAdjust(LimelightSubsystem subsystem) {
    limelight = subsystem;
    addRequirements(subsystem);
  }
  

  @Override
  public void execute() {
    adjust();
  }

  //adjust turret
  private void adjust() {
    String dir = limelight.direction();
    double sig = (Math.pow(Math.E, -0.15*(((limelight.getX())/2)-10)));
    SmartDashboard.putString("Turret Intended Direction",dir);
    if(limelight.getX() < -Constants.LIMELIGHT_AIM_TOLERANCE) {
      // target on left
      turretPower = 0.25;
    } else if(limelight.getX() > Constants.LIMELIGHT_AIM_TOLERANCE) {
      // target on right
      turretPower = -0.25;;
    } else if(limelight.getTargetVisible()) {
      // target in center
        if((limelight.getX() < -1)) {
          turretPower = 1/(1 + sig);
          SmartDashboard.putNumber("Turret Power", turretPower);
        } else if((limelight.getX() > 1)) {
          turretPower = -1/(1 + sig);
          SmartDashboard.putNumber("Turret Power", turretPower);
        }
        else {
          turretPower = 0;
          SmartDashboard.putNumber("Turret Power", turretPower);
        }

    } else  {
      // no target present
      turretPower = TurretCommand.range * Constants.SEEKING_POWER;
    }
  }
  
  
  public double getTurretPower() {
    return turretPower;
  }
}
