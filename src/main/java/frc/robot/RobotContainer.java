package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
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

  // Controllers and ports
  final XboxController driverController = new XboxController(0);
  final XboxController operatorController = new XboxController(1);

  // Defining doublesuppliers that we will use for axis
  private DoubleSupplier throttle = () -> driverController.getLeftY();
  private DoubleSupplier turn = () -> driverController.getRightX();
  private DoubleSupplier intakePower = () -> driverController.getLeftTriggerAxis()* 0.95 - driverController.getRightTriggerAxis() * 0.95;
  private DoubleSupplier chimneyPower = () -> intakePower.getAsDouble() * 0.9;
  // private DoubleSupplier turretBackupPower = () -> operatorController.getLeftTriggerAxis()* 0.4 - operatorController.getRightTriggerAxis() * 0.4;

  // Defining trigger classes for drive and feed (analog inputs)
  final Trigger drive = new JoystickButton(driverController, Axis.kLeftY.value)
  .or(new JoystickButton(driverController, Axis.kRightX.value));
  final Trigger intakeTrigger = new JoystickButton(driverController, Axis.kLeftTrigger.value)
  .or(new JoystickButton(driverController, Axis.kRightTrigger.value));

  // Backup turret trigger if limelight dies
  // final Trigger turretBackup = new JoystickButton(operatorController, Axis.kLeftTrigger.value)
  // .or(new JoystickButton(operatorController, Axis.kRightTrigger.value));
  
  // defining joystick buttons for other subsystems (digital input)
  final JoystickButton launchButton = new JoystickButton(driverController, Button.kY.value);
  final JoystickButton lowerFrontFeeder = new JoystickButton(driverController, Button.kLeftBumper.value);
  final JoystickButton lowerBackFeeder = new JoystickButton(driverController, Button.kRightBumper.value);

  // backup joystick buttons if limelight dies
  final JoystickButton launchbuttonBackup = new JoystickButton(driverController, Button.kA.value);
  final JoystickButton indexerbuttonBackup = new JoystickButton(driverController, Button.kX.value);

  // defining subsystems
  private final LimelightSubsystem limelight = new LimelightSubsystem();
  private final LauncherSubsystem launcher = new LauncherSubsystem();
  private final TurretSubsystem turret = new TurretSubsystem();
  private final IndexerSubsystem indexer = new IndexerSubsystem();
  private final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();
  private final ChimneySubsystem chimney = new ChimneySubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();
  
  // defining premeditatied commands
  private final LimelightCommand aimCommand = new LimelightCommand(limelight);
  private final IntakeCommand intakeCommand = new IntakeCommand(intake, intakePower, false, false);
  

  /* The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    SmartDashboard.putNumber("Launcher Velocity", 0.0);

    // default commands for functions
    drivetrain.setDefaultCommand(new DrivetrainCommand(drivetrain, throttle, turn));
    limelight.setDefaultCommand(aimCommand);
    turret.setDefaultCommand(new TurretCommand(turret, () -> 0, () -> false));
    indexer.setDefaultCommand(new IndexerCommand(indexer, () -> 0));
    launcher.setDefaultCommand(new LauncherCommand (launcher, () -> SmartDashboard.getNumber("Launcher Velocity", 0.0)));
    chimney.setDefaultCommand(new ChimneyCommand(chimney, chimneyPower));
    intake.setDefaultCommand(intakeCommand);

  }
  
  private void configureButtonBindings() {
    // drive buttons
    drive.whenActive(new DrivetrainCommand(drivetrain, throttle, turn));

    /* parallel command that runs:
    turret aiming
    launcher rev
    launching a ball on target lock */
    launchButton.whileHeld(new ParallelCommandGroup(
      new TurretCommand(turret, () -> aimCommand.getTurretPower(), () -> aimCommand.getTurretSeek()),
      new LauncherCommand(launcher, () -> -108 * limelight.getY() + 12750),
      new IndexerCommand(indexer, () -> aimCommand.getKickerOn() && launcher.isReady() ? 1 : 0)));

    // backup turret control if limelight fails
   // turretBackup.whenActive(new TurretCommand(turret, turretBackupPower, () -> false));

    // backup kicker control if limelight fails
    indexerbuttonBackup.whileHeld(new IndexerCommand(indexer, () -> 1));

    // backup launcher control if limelight fails
    launchbuttonBackup.whileHeld(new LauncherCommand(launcher, () -> 4000));

    // feeding button
    intakeTrigger.whenActive(new ParallelCommandGroup(
      new IntakeCommand(intake, intakePower, true, true),
      new ChimneyCommand(chimney, chimneyPower)));

    // toggles that run when the intakes needs to be lowered
    lowerFrontFeeder.toggleWhenPressed(
      new StartEndCommand(
        () -> intakeCommand.setFrontDown(true), 
        () -> intakeCommand.setFrontDown(false)));

    lowerBackFeeder.toggleWhenPressed(
      new StartEndCommand(
        () -> intakeCommand.setRearDown(true), 
        () -> intakeCommand.setRearDown(false)));
    }

  public Command getAutonomousCommand() {
    return null;
  }
}

// PID Drivetrain values
// tu = 210
// ku = 0.14
// P = 0.084
// I = 0.0008
// D = 2.205
