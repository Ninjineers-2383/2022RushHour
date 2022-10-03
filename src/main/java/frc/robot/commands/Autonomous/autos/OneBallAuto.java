package frc.robot.commands.Autonomous.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.Turret;
import frc.robot.commands.KickerCommand;
import frc.robot.commands.LauncherCommand;
import frc.robot.commands.TurretPositionCommand;
import frc.robot.commands.TurretSeekCommand;
import frc.robot.commands.Autonomous.AutoForward;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class OneBallAuto extends SequentialCommandGroup {

    public OneBallAuto(DrivetrainSubsystem drivetrain,
            KickerSubsystem kicker, LauncherSubsystem launcher, LimelightSubsystem limelight,
            TurretSubsystem turret) {
        // Run path following command, then stop at the end.
        addCommands(
                new TurretPositionCommand(turret, limelight, Turret.OFFSET_TICKS).withTimeout(0.5),
                new ParallelCommandGroup( // Intake system activate and intake first ball
                        new LauncherCommand(launcher, () -> 15200, () -> false).withTimeout(0.1),
                        new AutoForward(drivetrain, 5.3, 2, 0.75, 5),
                        new WaitCommand(0.5)),
                new ParallelCommandGroup( // Shoot two balls after feeding one
                        new LauncherCommand(launcher, () -> limelight.getLaunchingVelocity(), () -> false)
                                .withTimeout(0.9),
                        new TurretSeekCommand(turret, () -> limelight.getTurretPower() * 1.5,
                                () -> limelight.getTurretSeek(), false)
                                .withTimeout(1.2),
                        new SequentialCommandGroup(
                                new WaitCommand(0.3),
                                new KickerCommand(kicker, () -> 0.75)
                                        .withTimeout(2))));
    }
}
