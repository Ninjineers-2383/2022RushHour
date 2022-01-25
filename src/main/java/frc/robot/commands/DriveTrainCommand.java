// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveTrainSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;


/** An example command that uses an example subsystem. */
public class DriveTrainCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveTrainSubsystem driveTrainSubsystem;
  private DoubleSupplier throttle;
  private DoubleSupplier turn;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveTrainCommand(DriveTrainSubsystem subsystem, DoubleSupplier throttle, DoubleSupplier turn) {
    driveTrainSubsystem = subsystem;
    this.throttle = throttle;
    this.turn = turn;
    // Use addRequirements() here to declare subsystem dependencies.
    
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // See DriveTrainSubsystem.java for more details how the arcade() method works.
    driveTrainSubsystem.arcade(throttle.getAsDouble(), turn.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
