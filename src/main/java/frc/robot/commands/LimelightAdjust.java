package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.robot.subsystems.TurretSubsystem;

public class LimelightAdjust extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final LimelightSubsystem limelight;
  

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public LimelightAdjust(LimelightSubsystem subsystem) {
    limelight = subsystem;
    addRequirements(subsystem);
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
      RobotContainer.direction = 0;
      RobotContainer.launch = true;
      return;
    } 
    else if(dir.equals("Left")) {
      RobotContainer.direction = 1;
      RobotContainer.launch = true;
      //System.out.println("Turret adjusted left.");
    } 
    else if(dir.equals("Right")) {
      RobotContainer.direction = -1;
      RobotContainer.launch = true;
     //System.out.println("Turret adjusted right.");
    } 
    else if(dir.equals("Not Found")) {
      RobotContainer.direction = TurretCommand.range * Constants.SEEKING_POWER;
      RobotContainer.launch = false;
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
