package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ChimneySubsystem;

public class ChimneyCommand extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    // Creates an instance of ChimneySubsystem
    private final ChimneySubsystem chimney;

    // Creates a double supplier for the power of the Chimney
    private final DoubleSupplier power;

    /**
     * Takes in the chimney subsystem and runs chimney actions
     * 
     * @param chimney instance of ChimneySubsystem
     * @param power   power of the chimney from -1 to 1
     */
    public ChimneyCommand(ChimneySubsystem chimney, DoubleSupplier power) {
        this.chimney = chimney;
        this.power = power;
        addRequirements(chimney);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // See ChimneySubsystem.java for more details.
        // 1 degree of rotation = 145.695364 ticks
        double d_power = power.getAsDouble();
        chimney.setPower(d_power);
    }

    // Called once when the command ends
    @Override
    public void end(boolean interrupted) {
        chimney.setPower(0.0);
    }
}
