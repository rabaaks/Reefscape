package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class Module {
    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();

    private final EncoderIO encoderIO;
    private final EncoderIOInputsAutoLogged encoderInputs = new EncoderIOInputsAutoLogged();

    private final int index;

    private final SimpleMotorFeedforward driveFeedforward;

    Rotation2d rawModuleHeading;

    public Module(ModuleConfig config, ModuleIO io, EncoderIO encoderIO, int index) {
        this.io = io;
        this.encoderIO = encoderIO;

        this.index = index;

        driveFeedforward = new SimpleMotorFeedforward(config.driveS(), config.driveV(), config.driveA());

        updateInputs();

        if (encoderInputs.connected) {
            rawModuleHeading = new Rotation2d(encoderInputs.position);
        } else {
            rawModuleHeading = new Rotation2d(inputs.turnPosition);
        }
        
        io.resetPosition(rawModuleHeading.getRadians());
    }

    public void updateInputs() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive/Modules/" + index, inputs);

        encoderIO.updateInputs(encoderInputs);
        Logger.processInputs("Drive/Encoders/" + index, encoderInputs);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            inputs.driveVelocity,
            getRotation()
        );
    }

    public void setState(SwerveModuleState state) {
        io.setDriveVelocity(state.speedMetersPerSecond, driveFeedforward.calculate(state.speedMetersPerSecond));
        io.setTurnPosition(state.angle.getRadians(), 0.0);
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            inputs.drivePosition,
            getRotation()
        );
    }

    public void setVoltages(double driveVoltage, double turnVoltage) {
        io.setDriveVelocity(0.0, driveVoltage);
        io.setTurnPosition(0.0, turnVoltage);
    }

    public Rotation2d getRotation() {
        return new Rotation2d(inputs.turnPosition);
    }
}
