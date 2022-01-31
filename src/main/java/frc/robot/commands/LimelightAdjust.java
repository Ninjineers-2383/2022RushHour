package frc.robot.commands;

import frc.robot.Constants.Limelight;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class LimelightAdjust extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final LimelightSubsystem limelight;
  private final TurretSubsystem turret;
  

  public LimelightAdjust(LimelightSubsystem limelight, TurretSubsystem turret) {
    this.limelight = limelight;
    this.turret = turret;
    addRequirements(limelight);
    addRequirements(turret);
  }
  

  @Override
  public void execute() {
    double sig = (Math.pow(Math.E, -0.15*(((limelight.getX())/2)-10)));

    if(limelight.getX() < -Limelight.LIMELIGHT_AIM_TOLERANCE) {
      // target on left
      turret.setPower(0.25);
    } else if(limelight.getX() > Limelight.LIMELIGHT_AIM_TOLERANCE) {
      // target on right
      turret.setPower(-0.25);
    } else if(limelight.getTargetVisible()) {
      // target in center
        if((limelight.getX() < -1)) {
          turret.setPower(1/(1 + sig));
        } else if((limelight.getX() > 1)) {
          turret.setPower(-1/(1 + sig));
        }
        else {
          turret.setPower(0.0);
        }

    } else  {
      // no target present
      turret.seek();
    }
  }
}
