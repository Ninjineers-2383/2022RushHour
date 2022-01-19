// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.sql.Driver;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.commands.TurretCommand;
import frc.robot.commands.LauncherCommand;
import frc.robot.commands.LimelightAdjust;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  private final LimelightSubsystem limelight = new LimelightSubsystem();

  private final LimelightAdjust m_limelightAdjust = new LimelightAdjust(limelight);

  final XboxController m_driverController = new XboxController(0);

  public final LauncherSubsystem m_launcher = new LauncherSubsystem();

  private final TurretSubsystem m_turret = new TurretSubsystem();

  public boolean launcherOn = false;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    CommandScheduler.getInstance().setDefaultCommand(limelight, m_limelightAdjust);
    SmartDashboard.putNumber("Velocity", 0.0);

    m_launcher.setDefaultCommand(new LauncherCommand(m_launcher, 
    () -> (launcherOn) ? SmartDashboard.getNumber("Velocity", 0.0): 0));

    m_turret.setDefaultCommand(new TurretCommand(m_turret,
    () -> m_driverController.getLeftTriggerAxis() * 0.5 - m_driverController.getRightTriggerAxis() * 0.5));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_limelightAdjust;
  }
}
