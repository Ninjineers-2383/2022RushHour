package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ChimneySubsystem;

public class ChimneyCommand extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private final ChimneySubsystem chimney;
    private final BooleanSupplier power;

    // Creates a command that takes in a subsystem and speed and runs specific
    // actions created in the subsystem.
    // In this case, a Chimney command that takes in the Chimney subsystem and runs
    // Chimney subsystem actions.
    public ChimneyCommand(ChimneySubsystem chimney, BooleanSupplier power) {
        this.chimney = chimney;
        this.power = power;
        addRequirements(chimney);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // See ChimneySubsystem.java for more details.
        // 1 degree of rotation = 145.695364 ticks
        boolean b_power = power.getAsBoolean();
        chimney.setPower(b_power ? 1.0 : 0.0);
    }

    @Override
    public void end(boolean interrupted) {
        chimney.setPower(0.0);
    }
}
