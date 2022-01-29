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
  /** Creates a new ExampleSubsystem. */
  public LimelightSubsystem() {
    tolerance = Constants.LIMELIGHT_AIM_TOLERANCE;
  }

  private boolean valid = false;
  private double x;
  private double y;
  private double tolerance = 18;
  private MedianFilter fx = new MedianFilter(5);
  private MedianFilter fy = new MedianFilter(1000);

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tv = table.getEntry("tv");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");

    //read values periodically
    x = fx.calculate(tx.getDouble(0.0));
    valid = tv.getDouble(1) != 0;
    y = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    
    //the line right below doesn't work for some reason, so I'm using the y value above without the filter.
    //y = fy.calculate(ty.getDouble(0.0));
    double area = ta.getDouble(0.0);

    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
    SmartDashboard.putNumber("findDistance() value", findDistance());
  }

  public double getX() {
    return x;
  }

  public double negativeX() {
    if(x > 0) {
      x = x * -1;
      return x;
    } else {
      return x;
    }
  }

  public double getY() {
    return y;
  }

  public double findDistance() {
    double t = Math.toRadians(Constants.LIMELIGHT_ANGLE + y);
    //return t;
    return (Constants.LIMELIGHT_HEIGHT_DIFFERENCE/Math.tan(t));
  }

  //returns direction to move ring turret; i.e. if right is returned, turret must turn right.
  public String direction() {
    if (!valid) {
      return "Not Found";
    } else {
      if (x < -tolerance) {
        return "Left";
      } else if (x > tolerance) {
        return "Right";
      }
      return "Locked On";
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
