package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class LimelightAdjust extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final LimelightSubsystem limelight;

  public double turretPower = 0;
  public double kickerPower = 0;
  
  // Lie
  public LimelightAdjust(LimelightSubsystem subsystem) {
    limelight = subsystem;
    
    addRequirements(subsystem);
  }

  public double getTurretPower() {
    return turretPower;
  }

  public double getKickerPower() {
    return kickerPower;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    adjust();
  }

  //adjust turret
  private void adjust() {
    String dir = limelight.direction();
    double sig = (Math.pow(Math.E, -0.15*(((limelight.getX())/2)-10)));
    SmartDashboard.putString("Turret Intended Direction",dir);
    //if limelight is within tolerance, break from adjust
    if(dir.equals("Locked On")) {
        if((limelight.getX() < -2)) {
          turretPower = 1/(1 + sig);
          SmartDashboard.putNumber("Turret Power", turretPower);
        } else if((limelight.getX() > 2)) {
          turretPower = -1/(1 + sig);
          SmartDashboard.putNumber("Turret Power", turretPower);
        }
        else {
          turretPower = 0;
          SmartDashboard.putNumber("Turret Power", turretPower);
          kickerPower = 0.5;
        }
    }
    else if(dir.equals("Left")) {
      turretPower = 0.35;
      //System.out.println("Turret adjusted left.");
    } 
    else if(dir.equals("Right")) {
      turretPower = -0.35;
     //System.out.println("Turret adjusted right.");
    } 
    else if(dir.equals("Not Found")) {
      turretPower = TurretCommand.range * Constants.SEEKING_POWER;
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
