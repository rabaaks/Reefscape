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

    private final int index;

    private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(driveS, driveV, driveA);

    public Module(ModuleIO io, int index) {
        this.io = io;
        this.index = index;
    }

    public void updateInputs() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive/Modules/" + index, inputs);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            inputs.driveVelocity,
            new Rotation2d(inputs.turnPosition)
        );
    }

    public void setState(SwerveModuleState state) {
        io.setDriveVelocity(state.speedMetersPerSecond, driveFeedforward.calculate(state.speedMetersPerSecond));
        io.setTurnPosition(state.angle.getRadians(), 0.0);
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            inputs.drivePosition,
            new Rotation2d(inputs.turnPosition)
        );
    }

    public void setVoltages(double driveVoltage, double turnVoltage) {
        io.setDriveVelocity(0.0, driveVoltage);
        io.setTurnPosition(0.0, turnVoltage);
    }
}
