// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LimelightSubsystem extends SubsystemBase {

  private final double TOLERANCE;
  private final MedianFilter filteredX = new MedianFilter(5);
  private final MedianFilter filteredY = new MedianFilter(1000);

  private boolean targetValid = false;
  private double targetX;
  private double targetY;
  

  /** Creates a new ExampleSubsystem. */
  public LimelightSubsystem() {
    TOLERANCE = Constants.LIMELIGHT_AIM_TOLERANCE;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tableValidTarget = table.getEntry("tv");
    NetworkTableEntry tableTargetX = table.getEntry("tx");
    NetworkTableEntry tableTargetY = table.getEntry("ty");

    //read values periodically
    targetX = filteredX.calculate(tableTargetX.getDouble(0));
    targetValid = tableValidTarget.getDouble(1) != 0;
    targetY = filteredY.calculate(tableTargetY.getDouble(0));



    //post to smart dashboard periodically
    SmartDashboard.putNumber("Target X", targetX);
    SmartDashboard.putNumber("Target Y", targetY);
    
  }

  public double getX() {
    return targetX;
  }

  public double getY() {
    return targetY;
  }

  //returns direction to move ring turret; i.e. if right is returned, turret must turn right.
  public String direction() {
    if (!targetValid) {
      return "Not Found";
    } else {
      if (targetX < -TOLERANCE) {
        return "Left";
      } else if (targetX > TOLERANCE) {
        return "Right";
      }
      return "Locked On";
    }
  }
}
