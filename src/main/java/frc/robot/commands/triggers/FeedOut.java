package frc.robot.commands.triggers;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ColorSensorSubsystem;

public class FeedOut extends Trigger {

    private final ColorSensorSubsystem colorSensor;

    public FeedOut(ColorSensorSubsystem colorSensor) {
        this.colorSensor = colorSensor;
    }

    @Override
    public boolean get() {
        if (colorSensor.colorCheck().equals(colorSensor.getOppositeTeamColor())) {
            return true;
        }
        return false;

    }
}
