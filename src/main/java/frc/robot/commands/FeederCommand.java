package frc.robot.commands;

import frc.robot.subsystems.FeederSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class FeederCommand extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    private final FeederSubsystem m_subsystem;
    private final DoubleSupplier m_speed;

    // Creates a command that takes in a subsystem and speed and runs specific actions created in the subsystem.
    // In this case, a feeder command that takes in the feeder subsystem and runs feeder subsystem actions.
    public FeederCommand(FeederSubsystem subsystem, DoubleSupplier speed) {
        m_subsystem = subsystem;
        m_speed = speed;
        addRequirements(subsystem);
    } 

    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // See FeederSubsystem.java for more details.
        // 1 degree of rotation = 145.695364 ticks
        m_subsystem.feed(m_speed.getAsDouble());
        //m_subsystem.kickV(m_speed.getAsDouble());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
