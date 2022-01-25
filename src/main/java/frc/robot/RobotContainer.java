// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.commands.TurretCommand;
import frc.robot.commands.KickerCommand;
import frc.robot.commands.LauncherCommand;
import frc.robot.commands.LimelightAdjust;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.*;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  private final LimelightSubsystem limelight = new LimelightSubsystem();

  final XboxController m_driverController = new XboxController(0);

  private final LauncherSubsystem m_launcher = new LauncherSubsystem();

  private final TurretSubsystem m_turret = new TurretSubsystem();

  private final LimelightAdjust m_adjust = new LimelightAdjust(limelight);

  private final KickerSubsystem m_kicker = new KickerSubsystem();

  private Button launchButton = new Button(() -> m_driverController.getRightBumper());

  private Button kickerButton = new Button(() -> m_driverController.getLeftBumper());

  //private Button JoystickX = new Button(() -> m_driverController.)

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    SmartDashboard.putNumber("Launcher Velocity", 0.0);
    SmartDashboard.putNumber("Direction", m_adjust.getTurretPower());
    
    limelight.setDefaultCommand(m_adjust);
    m_turret.setDefaultCommand(new TurretCommand(m_turret, () -> 0));
    m_kicker.setDefaultCommand(new KickerCommand(m_kicker, () -> 0));
    m_launcher.setDefaultCommand(new LauncherCommand (m_launcher, () -> SmartDashboard.getNumber("Launcher Velocity", 0.0)));

  }
  
  private void configureButtonBindings() {
    
    kickerButton.whenHeld(new KickerCommand(m_kicker, () -> 0.5));

    // launchButton.whenHeld( new ParallelCommandGroup(
    //   new LauncherCommand (m_launcher, () ->  -108 * limelight.getY() + 11000), 
    //   (new TurretCommand(m_turret, () -> m_adjust.getTurretPower())),
    //   (new KickerCommand(m_kicker, () -> m_kicker.getKickerPower()))));
    // was 24.05x + 10500

    //launchButton.whenHeld(new LauncherCommand (m_launcher, () -> (24.05 * (limelight.findDistance()) + 10500)));
    //launchButton.whenHeld(new LauncherCommand (m_launcher, () -> (-110 * limelight.getY() + 11067)));
  }

  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
