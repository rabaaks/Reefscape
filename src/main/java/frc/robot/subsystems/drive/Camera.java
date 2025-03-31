package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import frc.robot.subsystems.drive.DriveConstants.VisionResult;

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
        Logger.processInputs("Drive/Cameras/" + index, inputs);
    }

    public VisionResult[] getResults() {
        return inputs.visionResults;
    }
}
