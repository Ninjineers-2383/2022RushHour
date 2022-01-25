package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class KickerSubsystem extends SubsystemBase {

    public TalonSRX kicker;
    public static double kickerPower = 0.5;
    
    public KickerSubsystem() {
        kicker = new TalonSRX(RobotMap.KICKER_PORT);
    }

    public double getKickerPower() {
        SmartDashboard.putNumber("Kicker Power", kickerPower);
        return kickerPower;
      }

    public void kick(Double power) {
        kicker.set(ControlMode.PercentOutput, power);
    }
    
    public void kickV(Double velocity) {
        kicker.set(ControlMode.Velocity, velocity);
    }
}
