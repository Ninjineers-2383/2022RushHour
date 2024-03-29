package frc.robot.commands.AutomatedCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ChimneyCommand;
import frc.robot.commands.KickerCommand;
import frc.robot.commands.LauncherCommand;
import frc.robot.commands.TurretSeekCommand;
import frc.robot.subsystems.ChimneySubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class DoubleShotCommand extends SequentialCommandGroup {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    double launchVelocity = 0;

    // Creates a command that takes in a subsystem and speed and runs specific
    // actions created in the subsystem.
    public DoubleShotCommand(ChimneySubsystem chimney, TurretSubsystem turret,
            KickerSubsystem kicker, LauncherSubsystem launcher, LimelightSubsystem limelight) {
        addCommands(
                new LauncherCommand(launcher,
                        () -> limelight.getLaunchingVelocity(),
                        () -> true, false)
                        .withInterrupt(() -> launcher.isReady())
                        .withTimeout(0.3),
                new InstantCommand(() -> launchVelocity = limelight
                        .getLaunchingVelocity()),
                new ParallelDeadlineGroup(
                        new KickerCommand(kicker, () -> 0.8).withTimeout(2),
                        new ChimneyCommand(chimney, () -> 1).perpetually(),
                        new TurretSeekCommand(turret, () -> 0,
                                () -> false, false),
                        new LauncherCommand(launcher,
                                () -> launchVelocity,
                                () -> true)));
    }
}
