package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.DrivetrainSubsystem;



/** An example command that uses an example subsystem. */
public class PIDTuneCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  
  private final DrivetrainSubsystem drivetrainSubsystem;
  private DoubleSupplier kp;
  private DoubleSupplier ki;
  private DoubleSupplier kd;

  private double p_kp;
  private double p_ki;
  private double p_kd;

  private static Timer timer = new Timer();
  private static double motorOutput = 0;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public PIDTuneCommand(DrivetrainSubsystem subsystem, DoubleSupplier kp, DoubleSupplier ki, DoubleSupplier kd) {
    drivetrainSubsystem = subsystem;
    this.kp = kp;
    this.ki = ki;
    this.kd = kd;
    // Use addRequirements() here to declare subsystem dependencies.
    
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double c_kp = kp.getAsDouble();
    double c_ki = kp.getAsDouble();
    double c_kd = kp.getAsDouble();
    // if (c_kp != p_kp || c_ki != p_ki || c_kd != p_kd) {
    //     p_kp = c_kp;
    //     p_ki = c_ki;
    //     p_kd = c_kd;

    //     //Configure P values
    //     drivetrainSubsystem.rightMasterMotor  .config_kP(0, c_kp);
    //     drivetrainSubsystem.rightFollowerMotor.config_kP(0, c_kp);
    //     drivetrainSubsystem.leftMasterMotor   .config_kP(0, c_kp);
    //     drivetrainSubsystem.leftFollowerMotor .config_kP(0, c_kp);

    //     //Configure I values
    //     drivetrainSubsystem.rightMasterMotor  .config_kI(0, c_ki);
    //     drivetrainSubsystem.rightFollowerMotor.config_kI(0, c_ki);
    //     drivetrainSubsystem.leftMasterMotor   .config_kI(0, c_ki);
    //     drivetrainSubsystem.leftFollowerMotor .config_kI(0, c_ki);

    //     //Configure D values
    //     drivetrainSubsystem.rightMasterMotor  .config_kD(0, c_kd);
    //     drivetrainSubsystem.rightFollowerMotor.config_kD(0, c_kd);
    //     drivetrainSubsystem.leftMasterMotor   .config_kD(0, c_kd);
    //     drivetrainSubsystem.leftFollowerMotor .config_kD(0, c_kd);
    // }

    
     if (motorOutput < 1000) {
        motorOutput += 0.1;
    }
        drivetrainSubsystem.VelocityOutput((int) motorOutput,(int)  motorOutput);

    // See DriveTrainSubsystem.java for more details how the arcade() method works.
    // drivetrainSubsystem.drive(throttle.getAsDouble(), turn.getAsDouble());
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
