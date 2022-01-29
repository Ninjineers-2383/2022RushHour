package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class FeederSubsystem extends SubsystemBase {

    // create kicker instance that uses a TalonSRX motor controller.
    public TalonSRX Frontfeeder;
    public TalonSRX Backfeeder;
    
    // Kicker subsystem defined
    public FeederSubsystem() {

        Frontfeeder = new TalonSRX(RobotMap.FRONT_FEEDER_PORT);
        Backfeeder = new TalonSRX(RobotMap.BACK_FEEDER_PORT);

        Frontfeeder.setInverted(true);
        Backfeeder.setInverted(false);
        
    }

    // method that returns nothing (void) but sets the kiceer at a set power in the inverval of [-1, 1]
    public void feed(Double power) {
        Frontfeeder.set(ControlMode.PercentOutput, power);
        Backfeeder.set(ControlMode.PercentOutput, power);
    }
}
