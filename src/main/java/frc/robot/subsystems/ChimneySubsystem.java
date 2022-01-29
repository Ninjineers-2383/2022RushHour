package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ChimneySubsystem extends SubsystemBase {

    // create Chimney instance that uses a TalonSRX motor controller.
    public TalonSRX Chimney;
    public static double ChimneyPower = 0.5;
    
    // Chimney subsystem defined
    public ChimneySubsystem() {

        Chimney = new TalonSRX(RobotMap.CHIMNEY_PORT);
        Chimney.setInverted(true);
        
    }

    // method that returns a double of Chimney power.
    public double getChimneyPower() {
        SmartDashboard.putNumber("Chimney Power", ChimneyPower);
        return ChimneyPower;
      }

    // method that returns nothing (void) but sets the kiceer at a set power in the inverval of [-1, 1]
    public void elevateP(Double power) {
        Chimney.set(ControlMode.PercentOutput, power);
    }
    
    // method that returns nothing (void) but sets the Chimney at a set velocity.
    // public void elevateV(Double velocity) {
    //     Chimney.set(ControlMode.Velocity, velocity);
    // }
}
