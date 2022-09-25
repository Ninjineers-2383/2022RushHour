package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.KickerSubsystem;

public class KickerCommand extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private final KickerSubsystem kicker;
    private final DoubleSupplier power;

    // private double previousPower = Double.NaN;

    // Creates a command that takes in a subsystem and speed and runs specific
    // actions created in the subsystem.
    // In this case, a kicker command that takes in the kicker subsystem and runs
    // kicker subsystem actions.
    public KickerCommand(KickerSubsystem kicker, DoubleSupplier power) {
        this.kicker = kicker;
        this.power = power;
        addRequirements(kicker);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // See KickerSubsystem.java for more details.
        // 1 degree of rotation = 145.695364 ticks
        double d_power = power.getAsDouble();
        kicker.setPower(d_power);
        // m_subsystem.kickV(m_speed.getAsDouble());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        kicker.setPower(0.0);
    }
}