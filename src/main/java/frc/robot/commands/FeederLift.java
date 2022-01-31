package frc.robot.commands;

import frc.robot.subsystems.FeederSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;


public class FeederLift extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    private final FeederSubsystem m_feeder;


    // Creates a command that takes in a subsystem and speed and runs specific actions created in the subsystem.
    // In this case, a kicker command that takes in the kicker subsystem and runs kicker subsystem actions.
    public FeederLift(FeederSubsystem feeder) {
        m_feeder = feeder;
    } 


    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_feeder.toggleState();
    }
}
