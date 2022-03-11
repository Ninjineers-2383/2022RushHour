package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;


public class BrakeCoastSwitchCommand extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    private final DrivetrainSubsystem drivetrain;
    private final ClimberSubsystem climber;

    private boolean current = true;
    
    public BrakeCoastSwitchCommand(DrivetrainSubsystem drivetrain, ClimberSubsystem climber) {
        this.drivetrain = drivetrain;
        this.climber = climber;
    } 


    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        current = !current;
        drivetrain.switchBrakeCoast(current);
        climber.switchBrakeCoast(current);
    }
}
