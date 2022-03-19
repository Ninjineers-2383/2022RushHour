package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Drivetrain;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;

/** An example command that uses an example subsystem. */
public class AutoForwardAim extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private final DrivetrainSubsystem drivetrainSubsystem;

    private final double kP_HEADING_CORRECTION = 0.01; // Strength of heading correction
    private final double adjustedMaxOutput;

    final private double DISTANCE_TICKS;
    final private double ACCELERATION_INTERVAL;
    final private double TIMEOUT;
    final private Timer TIMER;
    final private CameraSubsystem camera;

    private double leftBias = 0;
    private double rightBias = 0;

    final private double[] OFFSET_TICKS = new double[2];

    final private double aimStartTime;

    private boolean done = false;
    private int profileState = 0; // Finite State Machine

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public AutoForwardAim(DrivetrainSubsystem subsystem, CameraSubsystem camera, double distanceFeet,
            double accelerationIntervalFeet,
            double maxOutput, double timeout) {
        drivetrainSubsystem = subsystem;
        this.adjustedMaxOutput = maxOutput - Math.signum(maxOutput) * (Drivetrain.ksPercent); // Friction

        this.DISTANCE_TICKS = (int) (distanceFeet * Drivetrain.TICKS_PER_FOOT);
        this.ACCELERATION_INTERVAL = (int) (accelerationIntervalFeet * Drivetrain.TICKS_PER_FOOT);

        this.TIMEOUT = timeout;
        this.TIMER = new Timer();
        this.TIMER.start();
        this.camera = camera;
        this.aimStartTime = -1;
        // Use addRequirements() here to declare subsystem dependencies.

        addRequirements(subsystem);
    }

    public AutoForwardAim(DrivetrainSubsystem subsystem, CameraSubsystem camera, double startAimAfter,
            double distanceFeet,
            double accelerationIntervalFeet,
            double maxOutput, double timeout) {
        drivetrainSubsystem = subsystem;
        this.adjustedMaxOutput = maxOutput - Math.signum(maxOutput) * (Drivetrain.ksPercent); // Friction

        this.DISTANCE_TICKS = (int) (distanceFeet * Drivetrain.TICKS_PER_FOOT);
        this.ACCELERATION_INTERVAL = (int) (accelerationIntervalFeet * Drivetrain.TICKS_PER_FOOT);

        this.TIMEOUT = timeout;
        this.TIMER = new Timer();
        this.TIMER.start();
        this.camera = camera;
        this.aimStartTime = startAimAfter;
        // Use addRequirements() here to declare subsystem dependencies.

        addRequirements(subsystem);
    }

    public AutoForwardAim(DrivetrainSubsystem subsystem, CameraSubsystem camera, double startAimAfter,
            double distanceFeet,
            double accelerationIntervalFeet,
            double maxOutput, double timeout, double leftBias, double rightBias) {
        drivetrainSubsystem = subsystem;
        this.adjustedMaxOutput = maxOutput - Math.signum(maxOutput) * (Drivetrain.ksPercent); // Friction

        this.DISTANCE_TICKS = (int) (distanceFeet * Drivetrain.TICKS_PER_FOOT);
        this.ACCELERATION_INTERVAL = (int) (accelerationIntervalFeet * Drivetrain.TICKS_PER_FOOT);

        this.TIMEOUT = timeout;
        this.TIMER = new Timer();
        this.TIMER.start();
        this.camera = camera;
        this.aimStartTime = startAimAfter;
        this.leftBias = leftBias;
        this.rightBias = rightBias;
        // Use addRequirements() here to declare subsystem dependencies.

        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        SmartDashboard.putBoolean("Auto Done", false);
        TIMER.reset();
        OFFSET_TICKS[0] = drivetrainSubsystem.getLeftPosition();
        OFFSET_TICKS[1] = drivetrainSubsystem.getRightPosition();

        camera.setPipeline(DriverStation.getAlliance() == Alliance.Red ? 0 : 1);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        System.out.println(profileState);
        double leftOutput = 0;
        double rightOutput = 0;

        double workingTicks = Math.abs((((drivetrainSubsystem.getLeftPosition() - OFFSET_TICKS[0])
                + (drivetrainSubsystem.getRightPosition() - OFFSET_TICKS[1]))) / 2.0);

        if (TIMER.get() > TIMEOUT) {
            profileState = 3;
        }

        switch (profileState) { // Piece-wise motion profile
            case 0: // Ramp Up
                final double a = workingTicks / (double) ACCELERATION_INTERVAL;

                // SmartDashboard.putNumber("")

                leftOutput = a;
                rightOutput = a;

                if (workingTicks > ACCELERATION_INTERVAL) {
                    profileState++;
                } else {
                    break;
                }

            case 1: // Max Voltage
                leftOutput = 1;
                rightOutput = 1;
                if (workingTicks > DISTANCE_TICKS - ACCELERATION_INTERVAL) {
                    profileState++;
                } else {
                    break;
                }

            case 2: // Ramp Down
                final double b = 1
                        - ((double) (workingTicks - DISTANCE_TICKS + ACCELERATION_INTERVAL))
                                / (double) ACCELERATION_INTERVAL;

                leftOutput = b;
                rightOutput = b;

                if (workingTicks >= DISTANCE_TICKS) {
                    profileState++;
                } else {
                    break;
                }

            case 3:
                done = true;
        }

        double error = ((camera.getValid() && TIMER.get() > aimStartTime && (Math.abs(camera.getX()) > 1) ? 1 : 0))
                * (kP_HEADING_CORRECTION * camera.getX());

        leftOutput += error;
        rightOutput -= error;

        leftOutput += leftBias;
        rightOutput += rightBias;

        leftOutput *= adjustedMaxOutput; // Multiply by max output
        rightOutput *= adjustedMaxOutput;

        leftOutput += Math.signum(adjustedMaxOutput) * (Drivetrain.ksPercent);
        rightOutput += Math.signum(adjustedMaxOutput) * (Drivetrain.ksPercent);

        SmartDashboard.putNumber("left", leftOutput);
        SmartDashboard.putNumber("right", rightOutput);

        drivetrainSubsystem.tankDrive(leftOutput, rightOutput);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("Auto Done", true);
        drivetrainSubsystem.tankDrive(0, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return done;
    }
}