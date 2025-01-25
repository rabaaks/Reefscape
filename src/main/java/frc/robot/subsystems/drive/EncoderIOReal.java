package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj.AnalogEncoder;

import com.ctre.phoenix6.hardware.Pigeon2;

public class EncoderIOReal implements EncoderIO {
    AnalogEncoder encoder;

    double offset = 0.0;

    public EncoderIOReal(int id, double offset) {
        encoder = new AnalogEncoder(id);
        this.offset = offset;
    }

    @Override
    public void updateInputs(EncoderIOInputs inputs) {
        inputs.connected = true;
        inputs.position = 2.0 * Math.PI * encoder.get() - offset;
    }
}
