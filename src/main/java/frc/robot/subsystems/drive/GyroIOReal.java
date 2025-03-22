package frc.robot.subsystems.drive;

import com.ctre.phoenix6.hardware.Pigeon2;

import frc.robot.subsystems.drive.DriveConstants.GyroConfig;

public class GyroIOReal implements GyroIO {
    Pigeon2 gyro;

    public GyroIOReal(GyroConfig config) { 
        gyro = new Pigeon2(config.gyroId());
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
