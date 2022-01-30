package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ChimneySubsystem extends SubsystemBase {

    // create motor instance that uses a TalonSRX motor controller.
    private final TalonSRX Motor;
    
    // Chimney subsystem constructor
    public ChimneySubsystem() {
        Motor = new TalonSRX(RobotMap.CHIMNEY_PORT);
        Motor.setInverted(true);
        
    }

    // method that returns nothing (void) but sets the kicker at a set power in the inverval of [-1, 1]
    public void setPower(Double power) {
        Motor.set(ControlMode.PercentOutput, power);
        SmartDashboard.putNumber("Chimney Power", power);
    }
}
