package frc.robot.commands.triggers;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class AutoShoot extends Trigger {

    private final BooleanSupplier lock;
    private final BooleanSupplier spin;

    public AutoShoot(BooleanSupplier lock, BooleanSupplier spin) {
        this.lock = lock;
        this.spin = spin;
    }

    @Override
    public boolean get() {
        SmartDashboard.putBoolean("lock Boolean", lock.getAsBoolean());
        SmartDashboard.putBoolean("spin Boolean", spin.getAsBoolean());
        // filter.calculate((lock.getAsBoolean() ? 0.5 : 0.0) + (spin.getAsBoolean() ?
        // 0.5 : 0)) > 0.9
        return lock.getAsBoolean() && spin.getAsBoolean();
    }
}
