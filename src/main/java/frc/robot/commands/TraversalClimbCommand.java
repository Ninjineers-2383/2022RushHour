package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystemNew;

public class TraversalClimbCommand extends CommandBase {

    private final ClimberSubsystemNew subsystem;
    private final DoubleSupplier climbPower;

    public TraversalClimbCommand(ClimberSubsystemNew subsystem, DoubleSupplier climbPower) {
        this.subsystem = subsystem;
        this.climbPower = climbPower;

        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        subsystem.setPower(climbPower.getAsDouble(), climbPower.getAsDouble());
    }
}
