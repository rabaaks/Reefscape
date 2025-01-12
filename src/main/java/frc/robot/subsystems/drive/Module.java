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

    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(driveS, driveV);

    private final int index;

    public Module(ModuleIO io, int index) {
        this.io = io;
        this.index = index;
    }

    public void updateInputs() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive/Module" + index, inputs);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            inputs.driveVelocity * wheelRadius, 
            new Rotation2d(inputs.turnPosition)
        );
    }

    public void setState(SwerveModuleState state) {
        double velocity = state.speedMetersPerSecond / wheelRadius;
        io.setDriveVelocity(velocity, feedforward.calculate(velocity));

        io.setTurnPosition(state.angle.getRadians());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            inputs.drivePosition * wheelRadius,
            new Rotation2d(inputs.turnPosition)
        );
    }
}
