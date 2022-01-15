package frc.robot.commands;

import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

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
    if(dir.equals("none")) {
      //System.out.println("Turret not adjusted.");
      return;
    } 
    else if(dir.equals("left")) {
      //System.out.println("Turret adjusted left.");
    } 
    else if(dir.equals("right")) {
     //System.out.println("Turret adjusted right.");
    } 
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
