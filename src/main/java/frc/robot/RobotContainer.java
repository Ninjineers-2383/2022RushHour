// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.commands.TurretCommand;
import frc.robot.commands.LauncherCommand;
import frc.robot.commands.LimelightAdjust;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.*;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  private final LimelightSubsystem limelight = new LimelightSubsystem();

  final XboxController m_driverController = new XboxController(0);

  public final LauncherSubsystem m_launcher = new LauncherSubsystem();

  private final TurretSubsystem m_turret = new TurretSubsystem();
  
  public static double direction = 0;

  public static boolean launch = false;

  public boolean launcherEnabled = false;

  private Button launchButton = new Button(() -> m_driverController.getRightBumper());

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    SmartDashboard.putNumber("Velocity", 1000.0);
    SmartDashboard.putNumber("Direction", direction);
    
    limelight.setDefaultCommand(new LimelightAdjust(limelight));
    m_turret.setDefaultCommand(new TurretCommand(m_turret, () -> 0));
    m_launcher.setDefaultCommand(new LauncherCommand (m_launcher, () -> 0));

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  
  private void configureButtonBindings() {
    //launchButton.toggleWhenPressed(new TurretCommand (m_turret, () -> 0.5));
    launchButton.whenHeld( new ParallelCommandGroup(
      new LauncherCommand (m_launcher, () -> SmartDashboard.getNumber("Velocity", 0.0)), 
      (new TurretCommand(m_turret, () -> direction))));
    
    //launchButton.whenHeld(new ParallelCommandGroup());

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
