package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class KickerSubsystem extends SubsystemBase {

    // create kicker instance that uses a TalonSRX motor controller.
    public TalonSRX kicker;

    // Kicker subsystem defined
    public KickerSubsystem() {

        kicker = new TalonSRX(RobotMap.KICKER_PORT);
        kicker.setInverted(true);
        
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Kicker Velocity", kicker.getSelectedSensorVelocity());
    }

    // method that returns nothing (void) but sets the kicker at a set power in the inverval of [-1, 1]
    public void kick(Double power) {
        kicker.set(ControlMode.PercentOutput, power);
        SmartDashboard.putNumber("Kicker power", power);
    }
    
    // method that returns nothing (void) but sets the kicker at a set velocity.
    public void kickV(Double velocity) {
        kicker.set(ControlMode.Velocity, velocity);
        System.out.println(velocity);
    }
}
