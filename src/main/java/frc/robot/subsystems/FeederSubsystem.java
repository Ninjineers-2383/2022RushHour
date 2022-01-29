package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class FeederSubsystem extends SubsystemBase {

    // create kicker instance that uses a TalonSRX motor controller.
    public TalonSRX Frontfeeder;
    public TalonSRX Backfeeder;

    private Boolean front_feeder_state;
    private Boolean back_feeder_state;
    private Solenoid solenoid1 = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.RobotMap.SOLENOID1_PORT);
    private Solenoid solenoid2 = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.RobotMap.SOLENOID2_PORT);
    
    // Kicker subsystem defined
    public FeederSubsystem() {

        Frontfeeder = new TalonSRX(RobotMap.FRONT_FEEDER_PORT);
        Backfeeder = new TalonSRX(RobotMap.BACK_FEEDER_PORT);

        Frontfeeder.setInverted(true);
        Backfeeder.setInverted(false);

        solenoid1.set(false);
        solenoid2.set(false);
        front_feeder_state = false;
        back_feeder_state = false;
        
    }

    public void periodic() {
        //output feeder solenoid status on smartdashboard
        SmartDashboard.putBoolean("front_feeder_state", front_feeder_state);
        SmartDashboard.putBoolean("back_feeder_state", back_feeder_state);

    }

    public void toggleState(String side) {
        if (side == "front") {
            solenoid1.toggle();
            front_feeder_state = !front_feeder_state;
        } else if (side == "back") {
            solenoid2.toggle();
            back_feeder_state = !back_feeder_state;
        }
    }

    public boolean getState(String side) {
        if (side == "front") {
            return front_feeder_state;
        } else if (side == "back") {
            return back_feeder_state;
        }
        return false;
    }

    // method that returns nothing (void) but sets the kiceer at a set power in the inverval of [-1, 1]
    public void feed(Double power) {
        Frontfeeder.set(ControlMode.PercentOutput, power);
        Backfeeder.set(ControlMode.PercentOutput, power);
    }
}
