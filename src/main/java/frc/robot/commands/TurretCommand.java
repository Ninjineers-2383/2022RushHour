package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

public class TurretCommand extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private final TurretSubsystem turret;
    private final DoubleSupplier power;
    private final BooleanSupplier seek;
    private final Boolean center;
    private final Boolean flipSeek;
    private final int position;
    private final BooleanSupplier shouldMove;
    private boolean done = false;

    public TurretCommand(TurretSubsystem turret, DoubleSupplier power, BooleanSupplier seek, boolean flipSeek,
            int position) {
        this.turret = turret;
        this.power = power;
        this.seek = seek;
        this.center = false;
        this.position = 6300;
        this.flipSeek = flipSeek;
        this.shouldMove = () -> true;
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        turret.seekDirection(flipSeek);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (this.shouldMove.getAsBoolean()) {
            // 1 degree of rotation = 145.695364 ticks
            if (center) {
                if (Math.abs(turret.getCurrentPosition() - position) > 300) {
                    turret.runToPosition(position);
                } else {
                    turret.setPower(0.0);
                    done = false;
                }
            } else {
                if (seek.getAsBoolean()) {
                    turret.seek();
                } else {
                    turret.setPower(power.getAsDouble());
                }
            }
        } else {
            turret.setPower(0.0);
        }
    }

    @Override
    public void end(boolean force) {
        turret.setPower(0.0);
    }

    @Override
    public boolean isFinished() {
        return done;
    }
}
