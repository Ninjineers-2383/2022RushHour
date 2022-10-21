package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

public class TurretPowerCommand extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private final TurretSubsystem turret;

    private final DoubleSupplier power;

    public TurretPowerCommand(TurretSubsystem turret, DoubleSupplier power) {
        this.turret = turret;
        this.power = power;
    }

    @Override
    public void execute() {
        turret.setPower(power.getAsDouble());

    }

    @Override
    public void end(boolean force) {
        turret.setPower(0.0);
    }
}
