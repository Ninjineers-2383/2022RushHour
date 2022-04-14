package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystemNew;

public class TraversalClimbAuto extends CommandBase {

    private final ClimberSubsystemNew subsystem;
    private final DoubleSupplier climbPower;
    private final double finalPosition;
    private final double startingPosition;

    public TraversalClimbAuto(ClimberSubsystemNew subsystem, DoubleSupplier climbPower, double finalPosition) {
        this.subsystem = subsystem;
        this.climbPower = climbPower;
        this.finalPosition = finalPosition;
        this.startingPosition = subsystem.getAverageEncoderPos();

        addRequirements(subsystem);
    }

    @Override
    public boolean isFinished() {
        if (subsystem.getAverageEncoderPos() - startingPosition >= finalPosition) {
            return true;
        }
        return false;
    }

    @Override
    public void execute() {
        subsystem.setPower(climbPower.getAsDouble(), climbPower.getAsDouble());
    }
}