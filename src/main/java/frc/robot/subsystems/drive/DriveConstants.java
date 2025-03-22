package frc.robot.subsystems.drive;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class DriveConstants {
    public record Config(
        double maxSpeed,
        double halfWidth
    ) {}

    public record GyroConfig(
        int gyroId
    ) {}

    public record ModuleConfig(
        double driveS,
        double driveV,
        double driveA,

        double turnS,
        double turnV,
        double turnA
    ) {}

    public record ModuleIOConfig(
        double driveP,
        double driveI,
        double driveD,

        double turnP,
        double turnI,
        double turnD,

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

    public static final Config protoConfig = new Config(
        4.0,
        Units.inchesToMeters(10.0)
    );

    public static final GyroConfig protoGyroConfig = new GyroConfig(
        9
    );

    public static final ModuleConfig protoModuleConfig = new ModuleConfig(
        0.0,
        2.0,
        0.0,

        0.0,
        0.0,
        0.0
    );

    public static final ModuleIOConfig protoModuleIOConfig = new ModuleIOConfig(
        0.3,
        0.0,
        0.0,

        1.0,
        0.0,
        0.0,

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

    public static final EncoderRealConfig protoFrontLeftEncoderRealConfig = new EncoderRealConfig(1, 4.3);
    public static final EncoderRealConfig protoFrontRightEncoderRealConfig = new EncoderRealConfig(0, 4.9);
    public static final EncoderRealConfig protoBackLeftEncoderRealConfig = new EncoderRealConfig(2, 3.9);
    public static final EncoderRealConfig protoBackRightEncoderRealConfig = new EncoderRealConfig(3, 3.6);
}
