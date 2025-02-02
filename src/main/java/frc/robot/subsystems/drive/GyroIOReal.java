package frc.robot.subsystems.drive;

import com.ctre.phoenix6.hardware.Pigeon2;

public class GyroIOReal implements GyroIO {
    Pigeon2 gyro;

    public GyroIOReal(int id) { 
        gyro = new Pigeon2(id);
        gyro.reset();
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = true;
        inputs.yawPosition = gyro.getRotation2d().getRadians() - Math.PI;
    }

    @Override
    public void reset() {
        gyro.reset();
    }
}
