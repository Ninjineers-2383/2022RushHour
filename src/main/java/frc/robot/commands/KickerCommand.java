package frc.robot.commands;

import frc.robot.subsystems.KickerSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class KickerCommand extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    private final KickerSubsystem m_subsystem;
    private final DoubleSupplier m_speed;

    public KickerCommand(KickerSubsystem subsystem, DoubleSupplier speed) {
        m_subsystem = subsystem;
        m_speed = speed;
        addRequirements(subsystem);
    } 

    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // 1 degree of rotation = 145.695364 ticks
        m_subsystem.kick(m_speed.getAsDouble());
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
