package frc.robot.subsystems.drive;

import static frc.robot.Constants.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class DriveConstants {
    public record Config(
        double maxSpeed,
        Translation2d[] translations,
        FeedbackConfig driveFeedback,
        FeedbackConfig turnFeedback,
        double maxDriveProfileVelocity,
        double maxDriveProfileAcceleration,
        double maxTurnProfileVelocity,
        double maxTurnProfileAcceleration,
        FeedforwardConfig moduleDriveFeedforward,
        FeedbackConfig moduleDriveFeedback,
        FeedbackConfig moduleTurnFeedback
    ) {}

    public record GyroConfig(
        int gyroId
    ) {}

    public record ModuleIOConfig(
        double wheelRadius,

        double driveGearing,
        double turnGearing
    ) {}

    public record ModuleSimConfig(
        DCMotor driveMotor,
        DCMotor turnMotor,

        double driveMOI,
        double turnMOI
    ) {}

    public record ModuleRealConfig(
        int driveMotorId,
        int turnMotorId
    ) {}

    public record EncoderRealConfig(
        int encoderId,
        double encoderOffset
    ) {}

    public record CameraIOConfig(
        Transform3d robotToCamera,
        AprilTagFieldLayout aprilTagFieldLayout
    ) {}

    public record CameraRealConfig(
        String name
    ) {}

    public static final Config protoConfig = new Config(
        4.0,
        new Translation2d[] {
                new Translation2d(0.254, 0.254),
                new Translation2d(-0.254, -0.254)/* ,*/
                // new Translation2d(-0.254, 0.254),
                // new Translation2d(-0.254, -0.254)
        },
        new FeedbackConfig(1, 0, 0),
        new FeedbackConfig(1, 0, 0),

        1.0,
        1.0,
        1.0,
        1.0,

        new FeedforwardConfig(
            0.0,
            0.0,
            2.0,
            0.0
        ),
        new FeedbackConfig(
            0.3,
            0.0,
            0.0
        ),
        new FeedbackConfig(
            1.0,
            0.0,
            0.0
        )
    );

    public static final GyroConfig protoGyroConfig = new GyroConfig(
        9
    );
    public static final ModuleIOConfig protoModuleIOConfig = new ModuleIOConfig(
        Units.inchesToMeters(2.0),

        5.14,
        12.8
    );

    public static final ModuleSimConfig protoModuleSimConfig = new ModuleSimConfig(
        DCMotor.getNEO(1),
        DCMotor.getNEO(1),

        0.02,
        0.005
    );

    public static final ModuleRealConfig protoFrontLeftModuleRealConfig = new ModuleRealConfig(1, 2);
    public static final ModuleRealConfig protoFrontRightModuleRealConfig = new ModuleRealConfig(3, 4);
    public static final ModuleRealConfig protoBackLeftModuleRealConfig = new ModuleRealConfig(5, 6);
    public static final ModuleRealConfig protoBackRightModuleRealConfig = new ModuleRealConfig(7, 8);

    public static final EncoderRealConfig protoFrontLeftEncoderRealConfig = new EncoderRealConfig(0, 1.2);
    public static final EncoderRealConfig protoFrontRightEncoderRealConfig = new EncoderRealConfig(1, 4.9);
    public static final EncoderRealConfig protoBackLeftEncoderRealConfig = new EncoderRealConfig(2, 3.9);
    public static final EncoderRealConfig protoBackRightEncoderRealConfig = new EncoderRealConfig(3, 3.6);
    
    public static final CameraIOConfig protoBackCameraIOConfig = new CameraIOConfig(
        new Transform3d(
            new Translation3d(0.0, 0.0, 1.0),
            new Rotation3d(0.0, 0.0, Math.PI)
        ),
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField)
    );

    public static final CameraIOConfig protoFrontCameraIOConfig = new CameraIOConfig(
        new Transform3d(
            new Translation3d(0.2, 0.0, 0.0),
            new Rotation3d()
        ),
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField)
    );

    public static final CameraRealConfig protoBackCameraRealConfig = new CameraRealConfig(
        "Camera_Module_v1_r"
    );

    public static final CameraRealConfig protoFrontCameraRealConfig = new CameraRealConfig(
        "Camera_Module_v1_l"
    );

    public record VisionResult(
        Pose2d pose,
        double time,
        Matrix<N3, N1> stdDevs
    ) {}
}
