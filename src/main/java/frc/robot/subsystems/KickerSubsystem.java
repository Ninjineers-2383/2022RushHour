package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class KickerSubsystem extends SubsystemBase {

    // create kicker instance that uses a TalonSRX motor controller.
    public TalonSRX kicker;
    public static double kickerPower = 0.5;
    
    // Kicker subsystem defined
    public KickerSubsystem() {
        kicker = new TalonSRX(RobotMap.KICKER_PORT);
    }

    // method that returns a double of kicker power.
    public double getKickerPower() {
        SmartDashboard.putNumber("Kicker Power", kickerPower);
        return kickerPower;
      }

    // method that returns nothing (void) but sets the kiceer at a set power in the inverval of [-1, 1]
    public void kick(Double power) {
        kicker.set(ControlMode.PercentOutput, power);
    }
    
    // method that returns nothing (void) but sets the kicker at a set velocity.
    public void kickV(Double velocity) {
        kicker.set(ControlMode.Velocity, velocity);
    }
}
