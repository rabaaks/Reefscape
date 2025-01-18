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

    private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(driveS, driveV, driveA);
    private final SimpleMotorFeedforward turnFeedforward = new SimpleMotorFeedforward(turnS, turnV, turnA);

    public Module(ModuleIO io) {
        this.io = io;
    }

    public void updateInputs(int index) {
        io.updateInputs(inputs);
        Logger.processInputs("Drive/Module" + index, inputs);
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
}
