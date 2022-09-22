package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CompressorSubsystem;

public class CompressorCommand extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private final CompressorSubsystem compressor;

    public CompressorCommand(CompressorSubsystem compressor) {
        this.compressor = compressor;
    }

    public void useCompressor() {
        compressor.countUp();
        if (compressor.getCount() == 1) {
            compressor.enableCompressor();
        }
    }

    public void stopUsingCompressor() {
        compressor.countDown();
        if (compressor.getCount() == 0) {
            compressor.disableCompressor();
        }
    }
}
