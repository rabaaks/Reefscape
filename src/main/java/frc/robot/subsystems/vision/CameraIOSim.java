package frc.robot.subsystems.vision;

import org.photonvision.simulation.PhotonCameraSim;

public class CameraIOSim implements CameraIO {
    private final PhotonCameraSim camera;

    public CameraIOSim() {
        camera = new PhotonCameraSim(null);
    }
}
