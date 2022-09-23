package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystemNew;

public class TraversalClimbManualCommand extends CommandBase {

    private final ClimberSubsystemNew climber;
    private final DoubleSupplier power;
    private final DoubleSupplier rightPower;
    private final BooleanSupplier separate;

    public TraversalClimbManualCommand(ClimberSubsystemNew climber, DoubleSupplier power, DoubleSupplier rightPower,
            BooleanSupplier separate) {
        this.climber = climber;
        this.power = power;
        this.rightPower = rightPower;
        this.separate = separate;

        addRequirements(climber);
    }

    @Override
    public void execute() {
        if (!separate.getAsBoolean()) {
            climber.setPower(power.getAsDouble(), power.getAsDouble());
        } else {
            climber.setPower(power.getAsDouble(), rightPower.getAsDouble());
        }
    }
}
