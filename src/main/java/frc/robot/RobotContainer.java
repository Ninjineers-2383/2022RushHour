package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ChimneyCommand;
import frc.robot.commands.ClimberCommand;
import frc.robot.commands.DoubleIntakeCommand;
import frc.robot.commands.DrivetrainCommand;
import frc.robot.commands.IndexerCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.LauncherCommand;
import frc.robot.commands.LimelightCommand;
import frc.robot.commands.TurretCommand;
import frc.robot.commands.Autonomous.AutoForward;
import frc.robot.commands.Autonomous.AutoTurn;
import frc.robot.commands.Autonomous.LimelightCommandAuto;
import frc.robot.subsystems.ChimneySubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ColorSensorSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.commands.FeedOut;
import frc.robot.commands.FeedIn;


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
  //final JoystickButton launchButton = new JoystickButton(driverController, Button.kLeftBumper.value);
  final JoystickButton launchLowButton = new JoystickButton(driverController, Button.kA.value);
  // backup joystick buttons if limelight dies
  // final JoystickButton launchbuttonBackup = new JoystickButton(driverController, Button.kA.value);
  final JoystickButton indexerUp = new JoystickButton(driverController, Button.kLeftBumper.value);
  final JoystickButton indexerUpTwoBall = new JoystickButton(driverController, Button.kRightBumper.value);
  final JoystickButton indexerDown = new JoystickButton(driverController, Button.kB.value);

  // Backup turret trigger if limelight dies
  final Trigger turretBackup = new JoystickButton(operatorController, Axis.kLeftTrigger.value)
  .or(new JoystickButton(operatorController, Axis.kRightTrigger.value));
  final JoystickButton lowerFrontFeeder = new JoystickButton(operatorController, Button.kLeftBumper.value);
  final POVButton limelightClimb = new POVButton(operatorController, 0, 0);
  final POVButton limelightTarget = new POVButton(operatorController, 270, 0);
  final POVButton limelightYeet = new POVButton(operatorController, 90, 0);
  final JoystickButton lowerBackFeeder = new JoystickButton(operatorController, Button.kRightBumper.value);
  final JoystickButton hookUp = new JoystickButton(operatorController, Button.kX.value);
  final JoystickButton hookDown = new JoystickButton(operatorController, Button.kB.value);
  final JoystickButton climberUp = new JoystickButton(operatorController, Button.kY.value);
  final JoystickButton climberDown = new JoystickButton(operatorController, Button.kA.value);
  //final JoystickButton brakeCoastSwitch = new JoystickButton(operatorController, Button.kStart.value);
  
  // Defining doublesuppliers that we will use for axis
  private DoubleSupplier throttle = () -> driverController.getLeftY();
  // private DoubleSupplier leftVoltsTest = () -> SmartDashboard.getNumber("L Volts Test", 0);
  // private DoubleSupplier rightVoltsTest = () -> SmartDashboard.getNumber("R Volts Test", 0);
  private DoubleSupplier turn = () -> driverController.getRightX();
  private DoubleSupplier intakePower = () -> driverController.getLeftTriggerAxis()* 0.95 - driverController.getRightTriggerAxis() * 0.95;
  private DoubleSupplier chimneyPower = () -> intakePower.getAsDouble() * 0.75;
  private DoubleSupplier climberPower = () -> climberUp.get() ? 1 : climberDown.get() ? -1 : 0;
  private DoubleSupplier hookPower = () -> hookUp.get() ? 0.5 : hookDown.get() ? -0.5 : -0.1;
  //private DoubleSupplier turretBackupPower = () -> operatorController.getLeftTriggerAxis()* 0.4 - operatorController.getRightTriggerAxis() * 0.4;

  // defining subsystems
  public final LimelightSubsystem limelight = new LimelightSubsystem();
  public final LauncherSubsystem launcher = new LauncherSubsystem();
  public final TurretSubsystem turret = new TurretSubsystem();
  public final IndexerSubsystem indexer = new IndexerSubsystem();
  public final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();
  public final ChimneySubsystem chimney = new ChimneySubsystem();
  public final IntakeSubsystem intake = new IntakeSubsystem();
  public final ClimberSubsystem climber = new ClimberSubsystem();
  //public final ColorSensorSubsystem colorSensor = new ColorSensorSubsystem(intake, chimney);
  
  // defining premeditatied commands
  private final LimelightCommand aimCommand = new LimelightCommand(limelight, () -> turret.getCurrentPosition(), () -> drivetrain.getAverageVelocity(), false);
  private final IntakeCommand intakeCommand = new IntakeCommand(intake, intakePower, false, false);
  private final ClimberCommand climberCommand = new ClimberCommand(climber, climberPower, hookPower);
  // private final BrakeCoastSwitchCommand brakeCoastSwitchCommand = new BrakeCoastSwitchCommand(drivetrain, climber);

  // Custom Triggers 
  // public final FeedIn pooperIn = new FeedIn(colorSensor);
  // public final FeedOut pooperOut = new FeedOut(colorSensor);
  
  // Auto Chooser
  SendableChooser<Command> autoChooser = new SendableChooser<>();
  String teamColor = "blue";


  /* The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    SmartDashboard.putNumber("Launcher Velocity", 0.0);
    SmartDashboard.putNumber("Auto Duration", 0.0);

    // default commands for functions
    drivetrain.setDefaultCommand(new DrivetrainCommand(drivetrain, throttle, turn));
    limelight.setDefaultCommand(aimCommand);
    turret.setDefaultCommand(new TurretCommand(turret, true, 6300));
    indexer.setDefaultCommand(new IndexerCommand(indexer, () -> 0));
    launcher.setDefaultCommand(new LauncherCommand (launcher, () -> SmartDashboard.getNumber("Launcher Velocity", 0.0)));
    chimney.setDefaultCommand(new ChimneyCommand(chimney, chimneyPower, intake));
    intake.setDefaultCommand(intakeCommand);
    climber.setDefaultCommand(climberCommand);
    SmartDashboard.putBoolean("Aim Active", false);

    SendableChooser<String> teamColorChooser = new SendableChooser<>();
    teamColorChooser.setDefaultOption("Blue", "blue");
    teamColorChooser.addOption("Red", "red");
    SmartDashboard.putData("teamColorChooser", teamColorChooser);
//colorSensor.setColor(teamColor);
    

    SetAutoCommands();

  }
  
  private void configureButtonBindings() {
    // drive buttons
    drive.whenActive(new DrivetrainCommand(drivetrain, throttle, turn));

    // pooperIn.whenActive(new SequentialCommandGroup(
    //   new DoubleIntakeCommand(intake, ()-> -1,() -> -1, true, false).withTimeout(0.1),
    //   new ChimneyCommand(chimney, () -> -1, intake).withTimeout(0.5),
    //   new DoubleIntakeCommand(intake, ()-> 0,() -> 0, true, false).withTimeout(0.05),
    //   new ChimneyCommand(chimney, () -> 0, intake).withTimeout(0.05)
    // ));

    // pooperOut.whenActive(new SequentialCommandGroup(
    //   new DoubleIntakeCommand(intake, ()-> -1,() -> -1, true, false).withTimeout(0.1),
    //   new ChimneyCommand(chimney, () -> -1, intake).withTimeout(0.5),
    //   new DoubleIntakeCommand(intake, ()-> 0,() -> 0, true, false).withTimeout(0.05),
    //   new ChimneyCommand(chimney, () -> 0, intake).withTimeout(0.05)
    // ));

    /* parallel command that runs:
    turret aiming
    launcher rev
    launching a ball on target lock */
    limelightTarget.whileHeld(new ParallelCommandGroup(
      new LauncherCommand(launcher, () -> limelight.getLaunchingVelocity()),
      new TurretCommand(turret, () -> aimCommand.getTurretPower(), () -> aimCommand.getTurretSeek()),
      new StartEndCommand(() -> SmartDashboard.putBoolean("Aim Active", true), () -> SmartDashboard.putBoolean("Aim Active", false)))
    );
      
    limelightYeet.whileHeld(new ParallelCommandGroup(
      new LauncherCommand(launcher, () -> limelight.getLaunchingVelocity() + 4000),
      new TurretCommand(turret, () -> aimCommand.getTurretPower(), () -> aimCommand.getTurretSeek())));
    //launchButton.whileHeld(new IndexerCommand(indexer, () -> aimCommand.getKickerOn() && launcher.isReady() ? 1 : 0) );

    
    //launchButton.whenPressed(new LauncherCommand(launcher, () -> 5500));

    // backup turret control if limelight fails
    // turretBackup.whenActive(new TurretCommand(turret, turretBackupPower, () -> false));

    // backup kicker control if limelight fails
    indexerUp.whileHeld(new IndexerCommand(indexer, () -> 1));
    indexerDown.whileHeld(new IndexerCommand(indexer, () -> -1));

    indexerUpTwoBall.whenPressed(new SequentialCommandGroup(
      new ChimneyCommand(chimney, () -> -0.4, intake).withTimeout(0.1),
      new IndexerCommand(indexer, () -> 0.75).withTimeout(0.2),
      new IndexerCommand(indexer, () -> 0).withTimeout(0.3),
      new ChimneyCommand(chimney, () -> -1, intake).withTimeout(0.1),
      new ChimneyCommand(chimney, () -> -0.5, intake).withTimeout(0.05),
      new IndexerCommand(indexer, () -> 0.75).withTimeout(0.2)
    ));

    // backup launcher control if limelight failss
    // TODO: Add backup buttons to DPad
    // launchbuttonBackup.whileHeld(new LauncherCommand(launcher, () -> 4000));

    //brakeCoastSwitch.whenPressed(brakeCoastSwitchCommand);

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
    return autoChooser.getSelected();
  }

  public void SetAutoCommands() {

    LimelightCommandAuto autoLimelight = new LimelightCommandAuto(limelight, () -> turret.getCurrentPosition(), () -> drivetrain.getAverageVelocity(), false);
    LimelightCommandAuto autoLimelight2 = new LimelightCommandAuto(limelight, () -> turret.getCurrentPosition(), () -> drivetrain.getAverageVelocity(), false);
    LimelightCommandAuto autoLimelight3 = new LimelightCommandAuto(limelight, () -> turret.getCurrentPosition(), () -> drivetrain.getAverageVelocity(), false);
    
    Command fourBallAuto = new SequentialCommandGroup(
      new ParallelCommandGroup(   // Intake system activate and intake first ball
        new LauncherCommand(launcher, () -> 15200).withTimeout(0.1),
        new IntakeCommand(intake, () -> -1, false, true).withTimeout(0.1),
        new ChimneyCommand(chimney, () -> -1, intake).withTimeout(0.1),
        new AutoForward(drivetrain, 5.3, 2, 0.88, 5)
      ),
      new ParallelCommandGroup(   // Shoot two ballsez after feeeding one
        new LauncherCommand(launcher, () -> limelight.getLaunchingVelocity()).withTimeout(0.6),
        new ParallelRaceGroup(
          autoLimelight,
          new TurretCommand(turret, () -> autoLimelight.getTurretPower(), () -> autoLimelight.getTurretSeek()).withTimeout(1.2)
        ),
        new SequentialCommandGroup(
          new WaitCommand(0.3), 
          new ChimneyCommand(chimney, () -> 0, intake).withTimeout(0.1),
          new IndexerCommand(indexer, () -> 0.75).withTimeout(0.4),
          new IndexerCommand(indexer, () -> 0).withTimeout(0.05),
          new ChimneyCommand(chimney, () -> -1, intake).withTimeout(0.3),
          new ChimneyCommand(chimney, () -> -0.5, intake).withTimeout(0.15),
          new IndexerCommand(indexer, () -> 0.75).withTimeout(0.4)
        )
      ),
      new ParallelCommandGroup(   // Stop launch system
        new TurretCommand(turret, () -> 0, () -> false).withTimeout(0.1),
        new LauncherCommand(launcher, () -> 0).withTimeout(0.1),
        new IndexerCommand(indexer, () -> 0).withTimeout(0.1),
        new ChimneyCommand(chimney, () -> -1, intake).withTimeout(0.1),
        new AutoTurn(drivetrain, 19, 8, -0.4, 5)
      ),    // drives back and intakes human player ball
      new AutoForward(drivetrain, 10.2, 2.5, 0.9, 5),
      new AutoTurn(drivetrain, 35, 10, 0.6, 5),
      new AutoForward(drivetrain, 2.3, 0.5, 0.6, 2),
      new WaitCommand(0.2),
      new ParallelCommandGroup(
        new LauncherCommand(launcher, () -> 16500).withTimeout(0.1),
        new AutoTurn(drivetrain, 29, 10, -0.6, 2)
      ),
      new ParallelRaceGroup(
        autoLimelight2,
        new TurretCommand(turret, () -> autoLimelight2.getTurretPower() * 0.5, () -> autoLimelight2.getTurretSeek()).withTimeout(1.2),
        new AutoForward(drivetrain, 13.5, 2, -0.88, 2)
      ),
      new ParallelCommandGroup(   // Shoot two ballsez
        new LauncherCommand(launcher, () -> limelight.getLaunchingVelocity() - 1000).withTimeout(2),
        new SequentialCommandGroup(
          new WaitCommand(0.4), 
          new ChimneyCommand(chimney, () -> 0, intake).withTimeout(0.1),
          new IndexerCommand(indexer, () -> 0.75).withTimeout(0.3),
          new IndexerCommand(indexer, () -> 0).withTimeout(0.1),
          new ChimneyCommand(chimney, () -> -1, intake).withTimeout(0.2),
          new ChimneyCommand(chimney, () -> 0, intake).withTimeout(0.05),
          new IndexerCommand(indexer, () -> 0.75).withTimeout(0.3)
        )
      )
    );

    Command twoBallAuto = new SequentialCommandGroup(
      new ParallelCommandGroup(   // Intake system activate and intake first ball
        new LauncherCommand(launcher, () -> 15200).withTimeout(0.1),
        new IntakeCommand(intake, () -> -1, false, true).withTimeout(0.1),
        new ChimneyCommand(chimney, () -> -1, intake).withTimeout(0.1),
        new AutoForward(drivetrain, 5.3, 2, 0.75, 5),
        new WaitCommand(0.5)
      ),
      new ParallelCommandGroup(   // Shoot two balls after feeeding one
        new LauncherCommand(launcher, () -> limelight.getLaunchingVelocity()).withTimeout(0.9),
        new ParallelRaceGroup(
          autoLimelight3,
          new TurretCommand(turret, () -> autoLimelight3.getTurretPower(), () -> autoLimelight3.getTurretSeek()).withTimeout(1.2)
        ),
        new SequentialCommandGroup(
          new WaitCommand(0.3), 
          new ChimneyCommand(chimney, () -> 0, intake).withTimeout(0.1),
          new IndexerCommand(indexer, () -> 0.75).withTimeout(0.5),
          new IndexerCommand(indexer, () -> 0).withTimeout(0.05),
          new ChimneyCommand(chimney, () -> -1, intake).withTimeout(0.3),
          new ChimneyCommand(chimney, () -> -0.5, intake).withTimeout(0.15),
          new IndexerCommand(indexer, () -> 0.75).withTimeout(0.5)
        )
      ),
      new ParallelCommandGroup(   // Stop launch system
        new TurretCommand(turret, () -> 0, () -> false).withTimeout(0.1),
        new LauncherCommand(launcher, () -> 0).withTimeout(0.1),
        new IndexerCommand(indexer, () -> 0).withTimeout(0.1),
        new ChimneyCommand(chimney, () -> -1, intake).withTimeout(0.1)
      ),
      new AutoTurn(drivetrain, 60, 10, 0.6, 6),
      new IntakeCommand(intake, () -> -1, true, false).withTimeout(0.1),
      new AutoForward(drivetrain, 5, 2, -0.6, 5),
      new TurretCommand(turret, true, -1000).withTimeout(0.6),

      new LauncherCommand(launcher, () -> 8000).withTimeout(1),
      new IndexerCommand(indexer, () -> 0.75).withTimeout(0.4)
    );

    Command nullAuto = null;

    Command oneBallAuto = new SequentialCommandGroup(
      new ParallelCommandGroup(   // Intake system activate and intake first ball
        new LauncherCommand(launcher, () -> 15200).withTimeout(0.1),
        new AutoForward(drivetrain, 5.3, 2, 0.75, 5),
        new WaitCommand(0.5)
      ),
      new ParallelCommandGroup(   // Shoot two balls after feeeding one
      new LauncherCommand(launcher, () -> limelight.getLaunchingVelocity()).withTimeout(0.9),
        new TurretCommand(turret, () -> aimCommand.getTurretPower() * 1.5, () -> aimCommand.getTurretSeek()).withTimeout(1.2),
        new SequentialCommandGroup(
          new WaitCommand(0.3), 
          new IndexerCommand(indexer, () -> 0.75).withTimeout(2)))
          );

      Command testAuto = new SequentialCommandGroup(
        new ParallelCommandGroup(   // Shoot two balls after feeeding one
          new LauncherCommand(launcher, () -> limelight.getLaunchingVelocity()).withTimeout(0.9),
            new TurretCommand(turret, () -> aimCommand.getTurretPower() * 1.5, () -> aimCommand.getTurretSeek()).withTimeout(1.2),
            new SequentialCommandGroup(
              new WaitCommand(0.3), 
              new IndexerCommand(indexer, () -> 0.75).withTimeout(2)))
      );

    autoChooser.setDefaultOption("Two Ball", twoBallAuto);
    autoChooser.addOption("Four Ball", fourBallAuto);
    autoChooser.addOption("No Auto", nullAuto);
    autoChooser.addOption("One Ball", oneBallAuto);
    autoChooser.addOption("Test Auto", testAuto);

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }
}

// PID Drivetrain values
// tu = 210
// ku = 0.14
// P = 0.084
// I = 0.0008
// D = 2.205
