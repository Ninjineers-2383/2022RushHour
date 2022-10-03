package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

public class TurretSeekCommand extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private final TurretSubsystem turret;
    private final DoubleSupplier power;
    private final BooleanSupplier seek;
    private final Boolean flipSeek;

    public TurretSeekCommand(TurretSubsystem turret, DoubleSupplier power, BooleanSupplier seek, boolean flipSeek) {
        this.turret = turret;
        this.power = power;
        this.seek = seek;
        this.flipSeek = flipSeek;
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        turret.seekDirection(flipSeek);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (seek.getAsBoolean()) {
            turret.seek();
        } else {
            turret.setPower(power.getAsDouble());
        }
    }

    @Override
    public void end(boolean force) {
        turret.setPower(0.0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
