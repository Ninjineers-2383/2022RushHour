package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystemNew;

public class TraversalClimbManualCommand extends CommandBase {

    private final ClimberSubsystemNew subsystem;
    private final DoubleSupplier power;
    private final DoubleSupplier rightPower;
    private final BooleanSupplier separate;

    public TraversalClimbManualCommand(ClimberSubsystemNew subsystem, DoubleSupplier power, DoubleSupplier rightPower,
            BooleanSupplier separate) {
        this.subsystem = subsystem;
        this.power = power;
        this.rightPower = rightPower;
        this.separate = separate;

        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        subsystem.setPower(power.getAsDouble(), power.getAsDouble());
    }
}
