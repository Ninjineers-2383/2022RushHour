package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberCommand extends CommandBase {

    private final ClimberSubsystem subsystem;
    private final DoubleSupplier climbPower;
    private final DoubleSupplier hookPower;

    public ClimberCommand(ClimberSubsystem subsystem, DoubleSupplier climbPower, DoubleSupplier hookPower) {
        this.subsystem = subsystem;
        this.climbPower = climbPower;
        this.hookPower = hookPower;

        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        subsystem.setPower(climbPower.getAsDouble(), climbPower.getAsDouble(), hookPower.getAsDouble());
    }
}