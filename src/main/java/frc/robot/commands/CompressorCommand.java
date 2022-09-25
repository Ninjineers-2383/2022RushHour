package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CompressorSubsystem;

public class CompressorCommand extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    // Creates an instance of CompressorSubsystem
    private final CompressorSubsystem compressor;

    /**
     * Takes an instance of a compressor and runs commands with it
     * 
     * @param compressor instance of CompressorSubsystem
     */
    public CompressorCommand(CompressorSubsystem compressor) {
        this.compressor = compressor;
    }

    /**
     * Called whenever the intakes use the compressor
     */
    public void useCompressor() {
        compressor.countUp();
    }

    /**
     * Called whenever the intakes stop using the compressor
     */
    public void stopUsingCompressor() {
        compressor.countDown();
    }
}
