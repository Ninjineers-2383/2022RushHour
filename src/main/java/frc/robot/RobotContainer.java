// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.commands.TurretCommand;
import frc.robot.commands.KickerCommand;
import frc.robot.commands.LauncherCommand;
import frc.robot.commands.Chimney;
import frc.robot.subsystems.ChimneySubsystem;
import frc.robot.commands.LimelightAdjust;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.commands.Drivetrain;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.commands.FeederCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.*;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class RobotContainer {

  // The robot's subsystems and commands are defined here, as well as controllers
  private final LimelightSubsystem limelight = new LimelightSubsystem();

  final XboxController m_driverController = new XboxController(0);

  final XboxController m_operatorController = new XboxController(1);

  private final LauncherSubsystem m_launcher = new LauncherSubsystem();

  private final TurretSubsystem m_turret = new TurretSubsystem();

  private final LimelightAdjust m_adjust = new LimelightAdjust(limelight);

  private final KickerSubsystem m_kicker = new KickerSubsystem();

  private final DrivetrainSubsystem m_drivetrain = new DrivetrainSubsystem();

  private final ChimneySubsystem m_chimney = new ChimneySubsystem();

  private final FeederSubsystem m_feeder = new FeederSubsystem();

  private Button kickerButton = new Button(() -> m_driverController.getLeftBumper());

  private DoubleSupplier throttle = () -> m_driverController.getLeftY();

  private DoubleSupplier turn = () -> m_driverController.getRightX();

  private DoubleSupplier feederPower = () -> m_driverController.getLeftTriggerAxis()* 0.95 - m_driverController.getRightTriggerAxis() * 0.95;

  private double chimneyPower = feederPower.getAsDouble() * 0.75;

  //private Button JoystickX = new Button(() -> m_driverController.)

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    SmartDashboard.putNumber("Launcher Velocity", 0.0);
    SmartDashboard.putNumber("Direction", m_adjust.getTurretPower());

    /* Driving method. Default command will suffice, 
    as setting multiple commands when it is in use or not is unnecessary and overcomplicated. */
    m_drivetrain.setDefaultCommand(new Drivetrain(m_drivetrain, throttle, turn));
    
    // Limelight default command. See LimelightAdjust.java -> public void adjust() for more details on how it works.
    limelight.setDefaultCommand(m_adjust);

    // Turret default command. See TurretCommand.java for more details on how it works.
    m_turret.setDefaultCommand(new TurretCommand(m_turret, () -> 0));

    // kicker default command. See KickerCommand.java for more details on how it works.
    m_kicker.setDefaultCommand(new KickerCommand(m_kicker, () -> 0));

    // launcher default command. See LauncherCommand.java for more details on how it works.
    m_launcher.setDefaultCommand(new LauncherCommand (m_launcher, () -> SmartDashboard.getNumber("Launcher Velocity", 0.0)));

    m_chimney.setDefaultCommand(new Chimney(m_chimney, () -> 0));
    
    m_feeder.setDefaultCommand(new ParallelCommandGroup(
      new FeederCommand(m_feeder, feederPower),
      new Chimney(m_chimney, () -> chimneyPower),
      new KickerCommand(m_kicker, () -> 0.45)
    ));
  }
  
  private void configureButtonBindings() {

    /* parallel command that runs the limelight adjust method that moves the turret
     and revs up the launcher command to a velocity specified in the LauncherCommand.java file. 
     Runs the kicker when target is locked on */

    // launchButton.whenHeld(new ParallelCommandGroup(
    //   new TurretCommand(m_turret, () -> m_adjust.getTurretPower()),
    //   new LauncherCommand (m_launcher, () -> SmartDashboard.getNumber("Launcher Velocity", 0.0) /* -108 * limelight.getY() + 11000 */)
    // ));

    // kicker button 
    kickerButton.whenHeld(new KickerCommand(m_kicker, () -> 0.5));

    // // Y button sucks balls up chimney, while A button shoots balls down Chimney
    // // Kicker shoots down to prevent the robot from shooting the ball accidentally
    // chimneyPowerUp.whenHeld(new ParallelCommandGroup(
    //   new ChimneyCommand(m_chimney, () -> 0.7),
    //   new KickerCommand(m_kicker, () -> -0.45)));

    // chimneyPowerDown.whenHeld(new ParallelCommandGroup(
    //   new ChimneyCommand(m_chimney, () -> -0.7),
    //   new KickerCommand(m_kicker, () -> -0.45)
    // ));

    //launchButton.whenHeld(new LauncherCommand (m_launcher, () ->  -108 * limelight.getY() + 11000));

  }

  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
