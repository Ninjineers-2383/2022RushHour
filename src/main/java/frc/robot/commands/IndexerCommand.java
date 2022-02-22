package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.IndexerSubsystem;

public class IndexerCommand extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    private final IndexerSubsystem indexer;
    private final DoubleSupplier power;

    // Creates a command that takes in a subsystem and speed and runs specific actions created in the subsystem.
    // In this case, a kicker command that takes in the kicker subsystem and runs kicker subsystem actions.
    public IndexerCommand(IndexerSubsystem indexer, DoubleSupplier power) {
        this.indexer = indexer;
        this.power = power;
        addRequirements(indexer);
        System.out.println("My ass");
    } 
    

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // See KickerSubsystem.java for more details.
        // 1 degree of rotation = 145.695364 ticks
        indexer.setPower(power.getAsDouble());
        //m_subsystem.kickV(m_speed.getAsDouble());
    }
}