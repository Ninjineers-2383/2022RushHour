package frc.robot.commands;

import frc.robot.subsystems.FeederSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class FeederPneumaticsCommand extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    FeederSubsystem m_feeder;

    // Creates a command that takes in a subsystem and speed and runs specific actions created in the subsystem.
    // In this case, a kicker command that takes in the kicker subsystem and runs kicker subsystem actions.
    public FeederPneumaticsCommand(FeederSubsystem feeder) {
        m_feeder = feeder;
    } 
    

    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_feeder.toggleState();
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
