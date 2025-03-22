package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj.AnalogEncoder;
import frc.robot.subsystems.drive.DriveConstants.EncoderRealConfig;

public class EncoderIOReal implements EncoderIO {
    AnalogEncoder encoder;

    double offset = 0.0;

    public EncoderIOReal(EncoderRealConfig config) {
        encoder = new AnalogEncoder(config.encoderId());
        this.offset = config.encoderOffset();
    }

    @Override
    public void updateInputs(EncoderIOInputs inputs) {
        inputs.connected = true;
        inputs.position = 2.0 * Math.PI * encoder.get() - offset;
    }
}
