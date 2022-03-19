package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ColorSensorSubsystem;

public class BarfOutCommand extends CommandBase {

    private final ColorSensorSubsystem colorSensor;

    public BarfOutCommand(ColorSensorSubsystem colorSensor) {
        this.colorSensor = colorSensor;
        addRequirements(colorSensor);
    }

    @Override
    public void execute() {
        colorSensor.shootOut();
    }
}
