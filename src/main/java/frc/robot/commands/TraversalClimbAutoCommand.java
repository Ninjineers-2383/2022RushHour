package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class TraversalClimbAutoCommand extends CommandBase {

    private final ClimberSubsystem climber;
    private final DoubleSupplier climbPower;
    private final double finalPosition;
    private final double startingPosition;

    public TraversalClimbAutoCommand(ClimberSubsystem climber, DoubleSupplier climbPower, double finalPosition) {
        this.climber = climber;
        this.climbPower = climbPower;
        this.finalPosition = finalPosition;
        this.startingPosition = climber.getAverageEncoderPos();

        addRequirements(climber);
    }

    @Override
    public boolean isFinished() {
        if (climber.getAverageEncoderPos() - startingPosition >= finalPosition) {
            return true;
        }
        return false;
    }

    @Override
    public void execute() {
        climber.setPower(climbPower.getAsDouble(), climbPower.getAsDouble());
    }
}