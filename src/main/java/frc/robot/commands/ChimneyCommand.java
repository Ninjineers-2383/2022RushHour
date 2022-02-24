package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ChimneySubsystem;
import frc.robot.subsystems.IntakeSubsystem;


public class ChimneyCommand extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    private final ChimneySubsystem chimney;
    private final DoubleSupplier power;
    private final IntakeSubsystem intake;

    // Creates a command that takes in a subsystem and speed and runs specific actions created in the subsystem.
    // In this case, a Chimney command that takes in the Chimney subsystem and runs Chimney subsystem actions.
    public ChimneyCommand(ChimneySubsystem chimney, DoubleSupplier power, IntakeSubsystem intake) {
        this.chimney = chimney;
        this.power = power;
        this.intake = intake;
        addRequirements(chimney);
    } 


    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // See ChimneySubsystem.java for more details.
        // 1 degree of rotation = 145.695364 ticks
        chimney.setPower(-power.getAsDouble());
        //m_subsystem.kickV(m_speed.getAsDouble());
    }
}
