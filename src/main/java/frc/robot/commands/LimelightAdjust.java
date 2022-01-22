package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class LimelightAdjust extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final LimelightSubsystem limelight;

  public double turretPower = 0;
  

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public LimelightAdjust(LimelightSubsystem subsystem) {
    limelight = subsystem;
    addRequirements(subsystem);
  }

  public double getTurretPower() {
    return turretPower;
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
    SmartDashboard.putString("Turret Intended Direction",dir);
    //if limelight is within tolerance, break from adjust
    if(dir.equals("Locked On")) {
      //System.out.println("Turret not adjusted.");
      // if (limelight.getX() > Constants.LIMELIGHT_AIM_TOLERANCE / 2) {
      //   turretPower = -0.02;
      // } else if (limelight.getX() < Constants.LIMELIGHT_AIM_TOLERANCE / 2) {
      //   turretPower = 0.02;
      // }
      // return;
        // if((limelight.getX() > 8)&&(limelight.getX() < -8)) {
        //   turretPower = 
        //   if((limelight.getX() > 3) || (limelight.getX() < -3)) {
        //     turretPower = ((limelight.getX()*limelight.getX()) - 6*(limelight.getX()) + 9)/(200);
        //   } else {
        //     turretPower = 0;
        //   }
        // }
        if((limelight.getX() > 1) || (limelight.getX() < -1)) {
          double sig = (Math.pow(Math.E, 0.1*((limelight.getX()/2)+24)));
          turretPower = 1/(1 + sig);
          SmartDashboard.putNumber("Aiming", turretPower);
          SmartDashboard.putNumber("Turret Jerking Off", sig);
        } else {
          turretPower = 0;
          SmartDashboard.putNumber("Ready to shoot", turretPower);
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
