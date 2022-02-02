package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.*;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

import frc.robot.subsystems.TurretSubsystem;
import frc.robot.commands.TurretCommand;
import frc.robot.commands.IndexerCommand;
import frc.robot.commands.LauncherCommand;
import frc.robot.commands.ChimneyCommand;
import frc.robot.subsystems.ChimneySubsystem;
import frc.robot.commands.LimelightCommand;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.commands.DrivetrainCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.IndexerSubsystem;


public class RobotContainer {

  // The robot's subsystems and commands are defined here, as well as controllers
  private final LimelightSubsystem limelight = new LimelightSubsystem();

  final XboxController driverController = new XboxController(0);
  final XboxController operatorController = new XboxController(1);

  private DoubleSupplier throttle = () -> driverController.getLeftY();
  private DoubleSupplier turn = () -> driverController.getRightX();
  private DoubleSupplier intakePower = () -> operatorController.getLeftTriggerAxis()* 0.95 - operatorController.getRightTriggerAxis() * 0.95;
  private DoubleSupplier chimneyPower = () -> intakePower.getAsDouble() * 0.7;
  
  private Button launchButton = new Button(() -> driverController.getRightBumper());
  private Button frontIntakeButton = new Button(() -> operatorController.getRightBumper());
  private Button rearIntakeButton = new Button(() -> operatorController.getLeftBumper());

  private final LauncherSubsystem launcher = new LauncherSubsystem();
  private final TurretSubsystem turret = new TurretSubsystem();
  private final IndexerSubsystem indexer = new IndexerSubsystem();
  private final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();
  private final ChimneySubsystem chimney = new ChimneySubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();
  
  private final LimelightCommand aimCommand = new LimelightCommand(limelight);
  private final IntakeCommand intakeCommand = new IntakeCommand(intake, intakePower, false, false);
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    SmartDashboard.putNumber("Launcher Velocity", 0.0);

    drivetrain.setDefaultCommand(new DrivetrainCommand(drivetrain, throttle, turn));
    limelight.setDefaultCommand(aimCommand);
    turret.setDefaultCommand(new TurretCommand(turret, () -> 0, () -> false));
    indexer.setDefaultCommand(new IndexerCommand(indexer, () -> 0));
    launcher.setDefaultCommand(new LauncherCommand (launcher, () -> SmartDashboard.getNumber("Launcher Velocity", 0.0)));
    chimney.setDefaultCommand(new ChimneyCommand(chimney, chimneyPower));
    intake.setDefaultCommand(intakeCommand);

  }
  
  private void configureButtonBindings() {
    /* parallel command that runs:
    turret aiming
    launcher rev
    launching a ball on target lock */

    launchButton.whileHeld(new ParallelCommandGroup(
      new TurretCommand(turret, () -> aimCommand.getTurretPower(), () -> aimCommand.getTurretSeek()),
      new LauncherCommand (launcher, () -> -108 * limelight.getY() + 13000),
      new IndexerCommand(indexer, () -> aimCommand.getKickerOn() && launcher.isReady() ? 1 : 0)));
    
    // toggles that run when the intakes needs to be lowered
    frontIntakeButton.toggleWhenPressed(
      new StartEndCommand(
        () -> intakeCommand.setFrontDown(true), 
        () -> intakeCommand.setFrontDown(false)));

    rearIntakeButton.toggleWhenPressed(
      new StartEndCommand(
        () -> intakeCommand.setRearDown(true), 
        () -> intakeCommand.setRearDown(false)));
    }

  public Command getAutonomousCommand() {
    return null;
  }
}
