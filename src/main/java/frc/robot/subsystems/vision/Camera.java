package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.subsystems.vision.CameraIO.CameraIOInputs;

public class Camera {
    private final CameraIO io;
    private final CameraIOInputsAutoLogged inputs = new CameraIOInputsAutoLogged();

    private final int index;

    public Camera(CameraIO io, int index) {
        this.io = io;
        this.index = index;
    }

    public void updateInputs() {
        io.updateInputs(inputs);
        Logger.processInputs("Vision/Cameras/" + index, inputs);
    }

    public Pose3d getPose() {
        return inputs.pose;
    }
}
