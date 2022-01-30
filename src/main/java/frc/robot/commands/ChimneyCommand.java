package frc.robot.commands;

import frc.robot.subsystems.ChimneySubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ChimneyCommand extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    private final ChimneySubsystem m_subsystem;
    private final DoubleSupplier m_speed;

    // Creates a command that takes in a subsystem and speed and runs specific actions created in the subsystem.
    // In this case, a Chimney command that takes in the Chimney subsystem and runs Chimney subsystem actions.
    public ChimneyCommand(ChimneySubsystem subsystem, DoubleSupplier speed) {
        m_subsystem = subsystem;
        m_speed = speed;
        addRequirements(subsystem);
    } 

    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // See ChimneySubsystem.java for more details.
        // 1 degree of rotation = 145.695364 ticks
        m_subsystem.setPower(m_speed.getAsDouble());
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
