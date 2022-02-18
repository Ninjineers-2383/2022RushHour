package frc.robot;

import java.util.function.DoubleSupplier;

import com.fasterxml.jackson.databind.cfg.ContextAttributes;
import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.*;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.PIDController;

import frc.robot.subsystems.TurretSubsystem;
import frc.robot.commands.TurretCommand;
import frc.robot.commands.Autonomous.DriveToFirstBall;
import frc.robot.commands.IndexerCommand;
import frc.robot.commands.LauncherCommand;
import frc.robot.Constants.Drivetrain;
import frc.robot.commands.ChimneyCommand;
import frc.robot.subsystems.ChimneySubsystem;
import frc.robot.commands.LimelightCommand;
import frc.robot.commands.PIDTuneCommand;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.commands.DrivetrainCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.DrivetrainVoltTest;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;


public class RobotContainer {

  // Controllers and ports
  final XboxController driverController = new XboxController(0);
  final XboxController operatorController = new XboxController(1);

  // Defining doublesuppliers that we will use for axis
  private DoubleSupplier throttle = () -> driverController.getLeftY();
  private DoubleSupplier leftVoltsTest = () -> SmartDashboard.getNumber("L Volts Test", 0);
  private DoubleSupplier rightVoltsTest = () -> SmartDashboard.getNumber("R Volts Test", 0);
  private DoubleSupplier turn = () -> driverController.getRightX();
  //private DoubleSupplier intakePower = () -> driverController.getLeftTriggerAxis()* 0.95 - driverController.getRightTriggerAxis() * 0.95;
  //private DoubleSupplier chimneyPower = () -> intakePower.getAsDouble() * 0.9;
  private Double driveVelocity = 0.0;
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
  //final JoystickButton lowerFrontFeeder = new JoystickButton(driverController, Button.kLeftBumper.value);
  //final JoystickButton lowerBackFeeder = new JoystickButton(driverController, Button.kRightBumper.value);

  // backup joystick buttons if limelight dies
  final JoystickButton launchbuttonBackup = new JoystickButton(driverController, Button.kA.value);
  final JoystickButton indexerbuttonBackup = new JoystickButton(driverController, Button.kX.value);

  // defining subsystems
  private final LimelightSubsystem limelight = new LimelightSubsystem();
  private final LauncherSubsystem launcher = new LauncherSubsystem();
  private final TurretSubsystem turret = new TurretSubsystem();
  private final IndexerSubsystem indexer = new IndexerSubsystem();
  private final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();
  //private final ChimneySubsystem chimney = new ChimneySubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();
  
  // defining premeditatied commands
  private final LimelightCommand aimCommand = new LimelightCommand(limelight);
  //private final IntakeCommand intakeCommand = new IntakeCommand(intake, intakePower, false, false);
  

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
    //chimney.setDefaultCommand(new ChimneyCommand(chimney, chimneyPower));
    //intake.setDefaultCommand(intakeCommand);

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
    // intakeTrigger.whenActive(new ParallelCommandGroup(
    //   new IntakeCommand(intake, intakePower, true, true),
    //   new ChimneyCommand(chimney, chimneyPower)));

    // toggles that run when the intakes needs to be lowered
    // lowerFrontFeeder.toggleWhenPressed(
    //   new StartEndCommand(
    //     () -> intakeCommand.setFrontDown(true), 
    //     () -> intakeCommand.setFrontDown(false)));

    // lowerBackFeeder.toggleWhenPressed(
    //   new StartEndCommand(
    //     () -> intakeCommand.setRearDown(true), 
    //     () -> intakeCommand.setRearDown(false)));
    }

  public Command getAutonomousCommand() {

    // drivetrain.tankDriveVolts(leftVoltsTest.getAsDouble(), rightVoltsTest.getAsDouble());
    
    // We don't know if we need the following two objects, but I'm leaving it just in case.
    // Create a voltage constraint to ensure we don't accelerate too fast
    // var autoVoltageConstraint =
    // new DifferentialDriveVoltageConstraint(
    //     new SimpleMotorFeedforward(
    //         Constants.Drivetrain.ksVolts,
    //         Constants.Drivetrain.kvVoltSecondsPerMeter,
    //         Constants.Drivetrain.kaVoltSecondsSquaredPerMeter),
    //         Constants.Drivetrain.kDriveKinematics,
    //     10);
    // // Create config for trajectory
    // TrajectoryConfig config =
    //     new TrajectoryConfig(
    //             Constants.AutoConstants.kMaxSpeedMetersPerSecond,
    //             Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    //         // Add kinematics to ensure max speed is actually obeyed
    //         .setKinematics(Constants.Drivetrain.kDriveKinematics)
    //         // Apply the voltage constraint
    //         .addConstraint(autoVoltageConstraint);

    // String trajectoryJSON = "/PathWeaver/output/StraightLinePath.wpilib.json";
    // Trajectory trajectory = new Trajectory();
    // Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
    // try {
    //   trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    // } catch (IOException ex) {
    //   DriverStation.reportError("POOP IN MY MOUTH " + trajectoryJSON, ex.getStackTrace());
    // } 
    
    // System.out.println(trajectory.toString());
  
    Trajectory straightLine = PathPlanner.loadPath("Straight Line", 0.8, 0.5); 
    SimpleMotorFeedforward a = new SimpleMotorFeedforward(Constants.Drivetrain.ksVolts,
    Constants.Drivetrain.kvVoltSecondsPerMeter,
    Constants.Drivetrain.kaVoltSecondsSquaredPerMeter);
    SmartDashboard.putNumber("Simple feedforward", a.calculate(0.5, 1));
    
    RamseteCommand ramseteCommand = new RamseteCommand(
        straightLine,
        drivetrain::getPose,
        new RamseteController(Constants.AutoConstants.kRamseteB, Constants.AutoConstants.kRamseteZeta),
         new SimpleMotorFeedforward(Constants.Drivetrain.ksVolts,
                                    Constants.Drivetrain.kvVoltSecondsPerMeter,
                                    Constants.Drivetrain.kaVoltSecondsSquaredPerMeter),
        Constants.Drivetrain.kDriveKinematics,
        drivetrain::getWheelSpeeds,
        //CHANGE THE PID VALUES
        new PIDController(Constants.Drivetrain.Motor_kP, Constants.Drivetrain.Motor_kI, 0),
        new PIDController(Constants.Drivetrain.Motor_kP, Constants.Drivetrain.Motor_kI, 0),
        
        // RamseteCommand passes volts to the callback
        drivetrain::tankDriveVolts,
        drivetrain
    );

    // Reset odometry to the starting pose of the trajectory.
    drivetrain.resetOdometry(straightLine.getInitialPose());

    //return ramseteCommand.andThen(() -> drivetrain.tankDriveVolts(0, 0));
    return new SequentialCommandGroup(
      new IntakeCommand(intake, () -> 1, true, true),
      new DriveToFirstBall(drivetrain)
      );
    //return null;
  }
}

// PID Drivetrain values
// tu = 210
// ku = 0.14
// P = 0.084
// I = 0.0008
// D = 2.205