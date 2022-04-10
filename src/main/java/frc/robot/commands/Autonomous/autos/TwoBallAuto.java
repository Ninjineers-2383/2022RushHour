package frc.robot.commands.Autonomous.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.Turret;
import frc.robot.commands.ChimneyCommand;
import frc.robot.commands.IndexerCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.LauncherCommand;
import frc.robot.commands.LimelightCommand;
import frc.robot.commands.TurretCommand;
import frc.robot.commands.Autonomous.AutoForward;
import frc.robot.subsystems.ChimneySubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class TwoBallAuto extends SequentialCommandGroup {

    public TwoBallAuto(DrivetrainSubsystem drivetrain, IntakeSubsystem intake, ChimneySubsystem chimney,
            IndexerSubsystem indexer, LauncherSubsystem launcher, LimelightSubsystem limelight, TurretSubsystem turret,
            LimelightCommand aimCommand) {
        // Run path following command, then stop at the end.
        addCommands(
                new TurretCommand(turret, Turret.OFFSET_TICKS).withTimeout(0.5),
                new ParallelCommandGroup( // Intake system activate and intake first ball
                        new LauncherCommand(launcher, () -> 15200).withTimeout(0.1),
                        new IntakeCommand(intake, () -> -1, false, true).withTimeout(0.1),
                        new ChimneyCommand(chimney, () -> -1).withTimeout(0.1),
                        new AutoForward(drivetrain, 5.3, 2, 0.75, 5),
                        new WaitCommand(0.5)),
                new ParallelCommandGroup( // Shoot two balls after feeding one
                        new LauncherCommand(launcher, () -> limelight.getLaunchingVelocity())
                                .withTimeout(0.9),
                        new ParallelRaceGroup(
                                new TurretCommand(turret,
                                        () -> aimCommand.getTurretPower(),
                                        () -> aimCommand.getTurretSeek())
                                        .withTimeout(1.2)),
                        new SequentialCommandGroup(
                                new WaitCommand(0.3),
                                new ChimneyCommand(chimney, () -> 0)
                                        .withTimeout(0.1),
                                new IndexerCommand(indexer, () -> 0.75)
                                        .withTimeout(0.5),
                                new IndexerCommand(indexer, () -> 0).withTimeout(0.05),
                                new ChimneyCommand(chimney, () -> -1)
                                        .withTimeout(0.3),
                                new ChimneyCommand(chimney, () -> -0.5)
                                        .withTimeout(0.15),
                                new IndexerCommand(indexer, () -> 0.75)
                                        .withTimeout(0.5))),
                new ParallelCommandGroup( // Stop launch system
                        new TurretCommand(turret, () -> 0, () -> false).withTimeout(0.1),
                        new LauncherCommand(launcher, () -> 0).withTimeout(0.1),
                        new IndexerCommand(indexer, () -> 0).withTimeout(0.1),
                        new ChimneyCommand(chimney, () -> -1).withTimeout(0.1)));
    }
}
