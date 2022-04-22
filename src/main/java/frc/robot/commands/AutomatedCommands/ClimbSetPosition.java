package frc.robot.commands.AutomatedCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystemNew;

public class ClimbSetPosition extends CommandBase {
    final ClimberSubsystemNew subsystem;
    final DoubleSupplier position;
    boolean done = false;

    public ClimbSetPosition(ClimberSubsystemNew subsystem, DoubleSupplier position) {
        this.subsystem = subsystem;
        this.position = position;
        addRequirements(subsystem);

    }

    @Override
    public void initialize() {
        subsystem.resetPosition();
    }

    @Override
    public void execute() {
        done = subsystem.runToPosition(position.getAsDouble(), true);
    }

    @Override
    public boolean isFinished() {
        return done;
    }
}
