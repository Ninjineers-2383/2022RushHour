package frc.robot.commands.triggers;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * A {@link Trigger} that is active when all shooter subsystems are locked on
 * the set point for at lest 0.2 seconds
 */
public class AutoShoot extends Trigger {

    private final BooleanSupplier lock;
    private final BooleanSupplier spin;
    private final DoubleSupplier driveSpeed;
    private Timer timer;
    private boolean lockedOn;

    public AutoShoot(BooleanSupplier lock, BooleanSupplier spin, DoubleSupplier driveSpeed) {
        this.lock = lock;
        this.spin = spin;
        this.driveSpeed = driveSpeed;
        this.timer = new Timer();
        lockedOn = false;
    }

    @Override
    public boolean get() {
        SmartDashboard.putBoolean("AutoShoot/Lock", lock.getAsBoolean());
        SmartDashboard.putBoolean("AutoShoot/Spin", spin.getAsBoolean());
        SmartDashboard.putNumber("AutoShoot/Timer", timer.get());
        if (lock.getAsBoolean() && spin.getAsBoolean() && Math.abs(driveSpeed.getAsDouble()) < 0.1) {
            if (!lockedOn) {
                timer.reset();
                timer.start();
                lockedOn = true;
            } else if (timer.get() > 0.2) {
                return true;
            }
        } else {
            lockedOn = false;
        }
        return false;
    }
}
