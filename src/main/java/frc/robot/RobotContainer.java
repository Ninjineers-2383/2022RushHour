package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.BrakeCoastSwitchCommand;
import frc.robot.commands.ChimneyCommand;
import frc.robot.commands.ClimberCommand;
import frc.robot.commands.DrivetrainCommand;
import frc.robot.commands.IndexerCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.LauncherCommand;
import frc.robot.commands.LimelightCommand;
import frc.robot.commands.TurretCommand;
import frc.robot.commands.Autonomous.AutoForward;
import frc.robot.commands.Autonomous.AutoTurn;
import frc.robot.subsystems.ChimneySubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.TurretSubsystem;


//hooks are front

public class RobotContainer {

  // Controllers and ports
  final XboxController driverController = new XboxController(0);
  final XboxController operatorController = new XboxController(1);

  // Defining trigger classes for drive and feed (analog inputs)
  final Trigger drive = new JoystickButton(driverController, Axis.kLeftY.value)
  .or(new JoystickButton(driverController, Axis.kRightX.value));
  final Trigger intakeTrigger = new JoystickButton(driverController, Axis.kLeftTrigger.value)
  .or(new JoystickButton(driverController, Axis.kRightTrigger.value));
  
  // defining joystick buttons for other subsystems (digital input)
  final JoystickButton launchButton = new JoystickButton(driverController, Button.kLeftBumper.value);
  final JoystickButton launchLowButton = new JoystickButton(driverController, Button.kA.value);
  // backup joystick buttons if limelight dies
  // final JoystickButton launchbuttonBackup = new JoystickButton(driverController, Button.kA.value);
  final JoystickButton indexerUp = new JoystickButton(driverController, Button.kRightBumper.value);
  final JoystickButton indexerDown = new JoystickButton(driverController, Button.kB.value);

  // Backup turret trigger if limelight dies
  final Trigger turretBackup = new JoystickButton(operatorController, Axis.kLeftTrigger.value)
  .or(new JoystickButton(operatorController, Axis.kRightTrigger.value));
  final JoystickButton lowerFrontFeeder = new JoystickButton(operatorController, Button.kLeftBumper.value);
  final JoystickButton lowerBackFeeder = new JoystickButton(operatorController, Button.kRightBumper.value);
  final JoystickButton hookUp = new JoystickButton(operatorController, Button.kX.value);
  final JoystickButton hookDown = new JoystickButton(operatorController, Button.kB.value);
  final JoystickButton climberUp = new JoystickButton(operatorController, Button.kY.value);
  final JoystickButton climberDown = new JoystickButton(operatorController, Button.kA.value);
  //final JoystickButton brakeCoastSwitch = new JoystickButton(operatorController, Button.kStart.value);


  // Defining doublesuppliers that we will use for axis
  private DoubleSupplier throttle = () -> -driverController.getLeftY();
  // private DoubleSupplier leftVoltsTest = () -> SmartDashboard.getNumber("L Volts Test", 0);
  // private DoubleSupplier rightVoltsTest = () -> SmartDashboard.getNumber("R Volts Test", 0);
  private DoubleSupplier turn = () -> driverController.getRightX();
  private DoubleSupplier intakePower = () -> driverController.getLeftTriggerAxis()* 0.95 - driverController.getRightTriggerAxis() * 0.95;
  private DoubleSupplier chimneyPower = () -> intakePower.getAsDouble() * 0.75;
  private DoubleSupplier climberPower = () -> climberUp.get() ? 0.5 : climberDown.get() ? -.5 : 0;
  private DoubleSupplier hookPower = () -> hookUp.get() ? 1 : hookDown.get() ? -1 : 0;
  private Double driveVelocity = 0.0;
  // private DoubleSupplier turretBackupPower = () -> operatorController.getLeftTriggerAxis()* 0.4 - operatorController.getRightTriggerAxis() * 0.4;

  // defining subsystems
  private final LimelightSubsystem limelight = new LimelightSubsystem();
  private final LauncherSubsystem launcher = new LauncherSubsystem();
  private final TurretSubsystem turret = new TurretSubsystem();
  private final IndexerSubsystem indexer = new IndexerSubsystem();
  private final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();
  private final ChimneySubsystem chimney = new ChimneySubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final ClimberSubsystem climber = new ClimberSubsystem();
  
  // defining premeditatied commands
  private final LimelightCommand aimCommand = new LimelightCommand(limelight, turret);
  private final IntakeCommand intakeCommand = new IntakeCommand(intake, intakePower, false, false);
  private final ClimberCommand climberCommand = new ClimberCommand(climber, climberPower, hookPower);
  private final BrakeCoastSwitchCommand brakeCoastSwitchCommand = new BrakeCoastSwitchCommand(drivetrain, climber);
  

  /* The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    SmartDashboard.putNumber("Launcher Velocity", 0.0);
    SmartDashboard.putNumber("L Volts Test", 0.0);
    SmartDashboard.putNumber("R Volts Test", 0.0);
    SmartDashboard.putNumber("R Velocity", driveVelocity);
    SmartDashboard.putNumber("R Kp", Constants.Drivetrain.Motor_kP);
    SmartDashboard.putNumber("R Ki", Constants.Drivetrain.Motor_kI);
    SmartDashboard.putNumber("R Kd", Constants.Drivetrain.Motor_kD);
    SmartDashboard.putNumber("Throttle", throttle.getAsDouble());
    SmartDashboard.putNumber("Turn", turn.getAsDouble());
    // default commands for functions
    // drivetrain.setDefaultCommand(new PIDTuneCommand(drivetrain,
    //   () -> SmartDashboard.getNumber("R Kp", 0.0),
    //   () -> SmartDashboard.getNumber("R Ki", 0.0),
    //   () -> SmartDashboard.getNumber("R Kd", 0.0)
    //   ));

    drivetrain.setDefaultCommand(new DrivetrainCommand(drivetrain, throttle, turn));
    // drivetrain.setDefaultCommand(new DrivetrainVoltTest(drivetrain, leftVoltsTest, rightVoltsTest));
    limelight.setDefaultCommand(aimCommand);
    turret.setDefaultCommand(new TurretCommand(turret, () -> 0, () -> false));
    indexer.setDefaultCommand(new IndexerCommand(indexer, () -> 0));
    launcher.setDefaultCommand(new LauncherCommand (launcher, () -> SmartDashboard.getNumber("Launcher Velocity", 0.0)));
    chimney.setDefaultCommand(new ChimneyCommand(chimney, chimneyPower, intake));
    intake.setDefaultCommand(intakeCommand);
    climber.setDefaultCommand(climberCommand);

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
      new LauncherCommand(launcher, () -> -162.577 * limelight.getY() + 14286),
      new IndexerCommand(indexer, () -> aimCommand.getKickerOn() && launcher.isReady() ? 1 : 0)));

    launchButton.whenPressed(new LauncherCommand(launcher, () -> 5500));

    // backup turret control if limelight fails
   // turretBackup.whenActive(new TurretCommand(turret, turretBackupPower, () -> false));

    // backup kicker control if limelight fails
    indexerUp.whileHeld(new IndexerCommand(indexer, () -> 1));
    indexerDown.whileActiveContinuous(new IndexerCommand(indexer, () -> -1));

    // backup launcher control if limelight fails
    // TODO: Add backup buttons to DPad
    // launchbuttonBackup.whileHeld(new LauncherCommand(launcher, () -> 4000));

    //brakeCoastSwitch.whenPressed(brakeCoastSwitchCommand);

    // feeding button
    intakeTrigger.whenActive(new ParallelCommandGroup(
      new IntakeCommand(intake, intakePower, true, true),
      new ChimneyCommand(chimney, chimneyPower, intake)));

    // toggles that run when the intakes needs to be lowered
    // lowerFrontFeeder.whenPressed()

    //7 rearDown
    //6 rearUp
    //1 frontDown
    //0 frontUp

    lowerFrontFeeder.toggleWhenPressed(
      new StartEndCommand(
        () -> intakeCommand.setFrontDown(false), 
        () -> intakeCommand.setFrontDown(true)));

    lowerBackFeeder.toggleWhenPressed(
      new StartEndCommand(
        () -> intakeCommand.setRearDown(false), 
        () -> intakeCommand.setRearDown(true)));
    }

  public Command getAutonomousCommand() {

    return new SequentialCommandGroup(
      new LauncherCommand(launcher, () -> 11500).withTimeout(1),
      new IndexerCommand(indexer, () -> 1).withTimeout(1), //make sure this power is right
      new LauncherCommand(launcher, () -> 0).withTimeout(1),
      new IndexerCommand(indexer, () -> 0).withTimeout(1),
      new ParallelCommandGroup(new IntakeCommand(intake, () -> -1, false, true).withTimeout(1),
      new ChimneyCommand(chimney, () ->-0.75, intake).withTimeout(1)),
      new AutoForward(drivetrain, 3, 1, 5, 7),
      new WaitCommand(1),
      new IntakeCommand(intake, () -> 0, false, true).withTimeout(1),
      new ChimneyCommand(chimney, () ->0, intake).withTimeout(1),
      new IntakeCommand(intake, () -> 0, false, false).withTimeout(1),
      new LauncherCommand(launcher, () -> 13500).withTimeout(1),
      new IndexerCommand(indexer, () -> 0.7).withTimeout(1), //make sure this power is right
      new LauncherCommand(launcher, () -> 0).withTimeout(1),
      new IndexerCommand(indexer, () -> 0).withTimeout(1));
    //return null;
  }
}

// PID Drivetrain values
// tu = 210
// ku = 0.14
// P = 0.084
// I = 0.0008
// D = 2.205