package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.TurretSubsystem;

public class TurretCommand extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    private final TurretSubsystem turret;
    private final DoubleSupplier speed;
    private final BooleanSupplier seek;
    private final Boolean center;


    public TurretCommand(TurretSubsystem turret, DoubleSupplier power, BooleanSupplier seek) {
        this.turret = turret;
        this.speed = power;
        this.seek = seek;
        this.center = false;
        addRequirements(turret);
    } 

    public TurretCommand(TurretSubsystem turret, boolean center) {
        this.turret = turret;
        this.speed = () -> 0;
        this.seek = () -> false;
        this.center = center;
        addRequirements(turret);
    } 
    


    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // 1 degree of rotation = 145.695364 ticks
        if (center) {
            turret.center();
        } else {
            if (seek.getAsBoolean()) {
                turret.seek();
            } else {
                turret.setPower(speed.getAsDouble());
            }
        }
    }
}
