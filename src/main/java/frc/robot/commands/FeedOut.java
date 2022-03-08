package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ColorSensorSubsystem;

public class FeedOut extends Trigger {

    private final ColorSensorSubsystem colorSensor;

    public FeedOut(ColorSensorSubsystem colorSensor) {
        this.colorSensor = colorSensor;
    }

    @Override
    public boolean get() {
        if (colorSensor.colorCheck().equals("red")) {
            System.out.println("feedOut");
            return true;
        }
        return false;
    
    }
}
