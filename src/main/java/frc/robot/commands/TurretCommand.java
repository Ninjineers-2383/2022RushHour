package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

public class TurretCommand extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private final TurretSubsystem turret;
    private final DoubleSupplier speed;
    private final BooleanSupplier seek;
    private final Boolean center;
    private final Boolean flipSeek;
    private final int position;
    private final BooleanSupplier shouldMove;
    private boolean done = false;

    public TurretCommand(TurretSubsystem turret, DoubleSupplier power, BooleanSupplier seek, boolean flipSeek) {
        this.turret = turret;
        this.speed = power;
        this.seek = seek;
        this.center = false;
        this.position = 6300;
        this.flipSeek = flipSeek;
        this.shouldMove = () -> true;
        addRequirements(turret);
    }

    public TurretCommand(TurretSubsystem turret, DoubleSupplier power, BooleanSupplier seek,
            BooleanSupplier shouldMove) {
        this.turret = turret;
        this.speed = power;
        this.seek = seek;
        this.center = false;
        this.position = 6300;
        this.flipSeek = false;
        this.shouldMove = shouldMove;
        addRequirements(turret);
    }

    public TurretCommand(TurretSubsystem turret, DoubleSupplier power, BooleanSupplier seek) {
        this.turret = turret;
        this.speed = power;
        this.seek = seek;
        this.center = false;
        this.position = 6300;
        this.flipSeek = false;
        this.shouldMove = () -> true;
        addRequirements(turret);
    }

    public TurretCommand(TurretSubsystem turret, int position) {
        this.turret = turret;
        this.speed = () -> 0;
        this.seek = () -> false;
        this.center = true;
        this.position = position;
        this.flipSeek = false;
        this.shouldMove = () -> true;
        addRequirements(turret);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (this.shouldMove.getAsBoolean()) {
            // 1 degree of rotation = 145.695364 ticks
            if (center) {
                if (Math.abs(turret.getCurrentPosition() - position) > 600) {
                    turret.runToPosition(position);
                } else {
                    done = true;
                }
            } else {
                if (seek.getAsBoolean()) {
                    turret.seek(flipSeek);
                } else {
                    turret.setPower(speed.getAsDouble());
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
