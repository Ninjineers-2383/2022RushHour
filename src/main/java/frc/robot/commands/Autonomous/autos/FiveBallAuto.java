package frc.robot.commands.Autonomous.autos;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PerpetualCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.TurretCommand;
import frc.robot.commands.AutomatedCommands.DoubleShotCommand;
import frc.robot.commands.AutomatedCommands.SeekCommand;
import frc.robot.commands.AutomatedCommands.StopLaunchCommand;
import frc.robot.subsystems.ChimneySubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class FiveBallAuto extends SequentialCommandGroup {
    private DrivetrainSubsystem drivetrain;

    public FiveBallAuto(DrivetrainSubsystem drivetrain, IntakeSubsystem rearIntake,
            ChimneySubsystem chimney,
            IndexerSubsystem indexer, LauncherSubsystem launcher, LimelightSubsystem limelight,
            TurretSubsystem turret) {

        this.drivetrain = drivetrain;

        Trajectory trajectory1 = new Trajectory();
        Trajectory trajectory2 = new Trajectory();
        Trajectory trajectory3 = new Trajectory();
        Trajectory trajectory4 = new Trajectory();

        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath();
            trajectory1 = TrajectoryUtil.fromPathweaverJson(trajectoryPath.resolve("output/First Ball.wpilib.json"));
            trajectory2 = TrajectoryUtil.fromPathweaverJson(trajectoryPath.resolve("output/Second Ball.wpilib.json"));
            trajectory3 = TrajectoryUtil
                    .fromPathweaverJson(trajectoryPath.resolve("output/Third and Fourth Ball.wpilib.json"));
            trajectory4 = TrajectoryUtil
                    .fromPathweaverJson(trajectoryPath.resolve("output/Final Shots.wpilib.json"));
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory", ex.getStackTrace());
        }

        final Trajectory traj1f = trajectory1;

        // Run path following command, then stop at the end.
        addCommands(
                new InstantCommand(
                        () -> {
                            // Reset odometry to the starting pose of the trajectory.
                            drivetrain.resetOdometry(traj1f.getInitialPose());
                        }),
                // TODO: May be the wrong intake
                new IntakeCommand(rearIntake, () -> true, false, true).withTimeout(0.1),

                new ParallelDeadlineGroup( // Intake system activate and intake first ball
                        getRamseteCommand(trajectory1),
                        new PerpetualCommand(new SeekCommand(launcher, limelight, turret, false))),

                new DoubleShotCommand(chimney, turret, indexer, launcher, limelight),

                new StopLaunchCommand(launcher, indexer, chimney, turret),
                // new IntakeCommand(intake, () -> -0.8, true, false).withTimeout(0.1),

                new IntakeCommand(rearIntake, () -> true, true, false).withTimeout(0.1),

                new ParallelDeadlineGroup(
                        getRamseteCommand(trajectory2),
                        // new PerpetualCommand(new SeekCommand(launcher, limelight, turret,
                        // aimCommand))
                        new TurretCommand(turret, 10500)),

                new SeekCommand(launcher, limelight, turret, false).withTimeout(0.4),

                new DoubleShotCommand(chimney, turret, indexer, launcher, limelight),

                new StopLaunchCommand(launcher, indexer, chimney, turret),

                getRamseteCommand(trajectory3),

                new WaitCommand(0.5),

                new ParallelDeadlineGroup(
                        getRamseteCommand(trajectory4),
                        new TurretCommand(turret, 19600)),

                new SeekCommand(launcher, limelight, turret, false).withTimeout(0.4),

                new DoubleShotCommand(chimney, turret, indexer, launcher, limelight),

                new StopLaunchCommand(launcher, indexer, chimney, turret),

                new InstantCommand(() -> drivetrain.tankDriveVolts(0, 0)));
    }

    private Command getRamseteCommand(Trajectory trajectory) {
        return new RamseteCommand(
                trajectory,
                drivetrain::getPose,
                new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
                new SimpleMotorFeedforward(
                        DriveConstants.ksVolts,
                        DriveConstants.kvVoltSecondsPerMeter,
                        DriveConstants.kaVoltSecondsSquaredPerMeter),
                DriveConstants.kDriveKinematics,
                drivetrain::getWheelSpeeds,
                new PIDController(DriveConstants.kPDriveVel, 0, 0),
                new PIDController(DriveConstants.kPDriveVel, 0, 0),
                // RamseteCommand passes volts to the callback
                drivetrain::tankDriveVolts,
                drivetrain);
    }
}
