package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ColorSensorSubsystem;

public class FeedIn extends Trigger {

    private final ColorSensorSubsystem colorSensor;

    public FeedIn(ColorSensorSubsystem colorSensor) {
        this.colorSensor = colorSensor;
    }

    @Override
    public boolean get() {
        if (colorSensor.colorCheck().equals(colorSensor.getTeamColor())) {
            return true;
        }
            return false;
    
    }
}
