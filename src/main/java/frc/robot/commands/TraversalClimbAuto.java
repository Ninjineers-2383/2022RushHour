package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Autonomous.AutoForward;
import frc.robot.subsystems.ClimberSubsystemNew;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class TraversalClimb extends SequentialCommandGroup {
    private DrivetrainSubsystem drivetrain;
    private IntakeSubsystem intake;
    private ClimberSubsystemNew climber;

    public TraversalClimb(DrivetrainSubsystem drivetrain, IntakeSubsystem intake, ClimberSubsystemNew climber) {

        this.drivetrain = drivetrain;
        this.intake = intake;
        this.climber = climber;

        // Run path following command, then stop at the end.
        addCommands(
                new AutoForward(drivetrain, 1.5, 1, 0.5, 2),
                new ClimberCommandAuto(climber, () -> -0.5, 100));
    }
}
