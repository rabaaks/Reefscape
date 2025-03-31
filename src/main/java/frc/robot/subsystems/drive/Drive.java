package frc.robot.subsystems.drive;

import static frc.robot.Constants.*;
import static frc.robot.subsystems.drive.DriveConstants.*;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {
    private final Module[] modules;

    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    private Rotation2d rawGyroRotation = new Rotation2d();
    
    private SwerveModulePosition[] previousPositions;

    private final SwerveDriveKinematics kinematics;
    private final SwerveDrivePoseEstimator poseEstimator;

    private final double maxSpeed;

    private final ProfiledPIDController xController;
    private final ProfiledPIDController yController;
    private final ProfiledPIDController zController;

    LoggedNetworkNumber driveP = new LoggedNetworkNumber("/Tuning/Drive/DriveP");
    LoggedNetworkNumber driveI = new LoggedNetworkNumber("/Tuning/Drive/DriveI");
    LoggedNetworkNumber driveD = new LoggedNetworkNumber("/Tuning/Drive/DriveD");

    LoggedNetworkNumber turnP = new LoggedNetworkNumber("/Tuning/Drive/TurnP");
    LoggedNetworkNumber turnI = new LoggedNetworkNumber("/Tuning/Drive/TurnI");
    LoggedNetworkNumber turnD = new LoggedNetworkNumber("/Tuning/Drive/TurnD");

    LoggedNetworkNumber moduleDriveP = new LoggedNetworkNumber("/Tuning/Modules/DriveP");
    LoggedNetworkNumber moduleDriveI = new LoggedNetworkNumber("/Tuning/Modules/DriveI");
    LoggedNetworkNumber moduleDriveD = new LoggedNetworkNumber("/Tuning/Modules/DriveD");
    LoggedNetworkNumber moduleDriveS = new LoggedNetworkNumber("/Tuning/Modules/DriveS");
    LoggedNetworkNumber moduleDriveV = new LoggedNetworkNumber("/Tuning/Modules/DriveV");
    LoggedNetworkNumber moduleDriveA = new LoggedNetworkNumber("/Tuning/Modules/DriveA");

    LoggedNetworkNumber moduleTurnP = new LoggedNetworkNumber("/Tuning/Modules/TurnP");
    LoggedNetworkNumber moduleTurnI = new LoggedNetworkNumber("/Tuning/Modules/TurnI");
    LoggedNetworkNumber moduleTurnD = new LoggedNetworkNumber("/Tuning/Modules/TurnD");

    private final Camera[] cameras;

    public Drive(Config config, GyroIO gyroIO, Module[] modules, Camera[] cameras) {
        this.gyroIO = gyroIO;
        this.modules = modules;
        this.cameras = cameras;

        driveP.setDefault(config.driveFeedback().p());
        driveI.setDefault(config.driveFeedback().i());
        driveD.setDefault(config.driveFeedback().d());

        turnP.setDefault(config.turnFeedback().p());
        turnI.setDefault(config.turnFeedback().i());
        turnD.setDefault(config.turnFeedback().d());

        moduleDriveP.setDefault(config.moduleDriveFeedback().p());
        moduleDriveI.setDefault(config.moduleDriveFeedback().i());
        moduleDriveD.setDefault(config.moduleDriveFeedback().d());

        moduleDriveP.setDefault(config.moduleDriveFeedback().p());
        moduleDriveI.setDefault(config.moduleDriveFeedback().i());
        moduleDriveD.setDefault(config.moduleDriveFeedback().d());
        moduleDriveS.setDefault(config.moduleDriveFeedforward().s());
        moduleDriveV.setDefault(config.moduleDriveFeedforward().v());
        moduleDriveA.setDefault(config.moduleDriveFeedforward().a());

        moduleTurnP.setDefault(config.moduleTurnFeedback().p());
        moduleTurnI.setDefault(config.moduleTurnFeedback().i());
        moduleTurnD.setDefault(config.moduleTurnFeedback().d());

        kinematics = new SwerveDriveKinematics(config.translations());

        previousPositions = new SwerveModulePosition[modules.length];
        for (int i = 0; i < modules.length; i++) {
            previousPositions[i] = new SwerveModulePosition();
        }
        poseEstimator = new SwerveDrivePoseEstimator(kinematics, new Rotation2d(), previousPositions, new Pose2d());

        maxSpeed = config.maxSpeed();

        gyroIO.reset();

        xController = new ProfiledPIDController(
            0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0)
        );

        yController = new ProfiledPIDController(
            0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0)
        );

        zController = new ProfiledPIDController(
            0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0)
        );

        zController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void periodic() {
        xController.setP(driveP.get());
        xController.setI(driveI.get());
        xController.setD(driveD.get());
        yController.setP(driveP.get());
        yController.setI(driveI.get());
        yController.setD(driveD.get());

        zController.setP(turnP.get());
        zController.setI(turnI.get());
        zController.setD(turnD.get());

        for (Module module : modules) {
            module.setConfig(
                new FeedbackConfig(moduleDriveP.get(), moduleDriveI.get(), moduleDriveD.get()),
                new FeedbackConfig(moduleTurnP.get(), moduleTurnI.get(), moduleTurnD.get()),
                new FeedforwardConfig(moduleDriveS.get(), 0.0, moduleDriveV.get(), moduleDriveA.get())
            );
            module.updateInputs();
        }

        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);

        SwerveModulePosition[] positions = getPositions();
        if (gyroInputs.connected) {
            rawGyroRotation = new Rotation2d(gyroInputs.yawPosition);
        } else {
            SwerveModulePosition[] deltas = new SwerveModulePosition[modules.length];
            for (int i = 0; i < modules.length; i++) {
                deltas[i] = new SwerveModulePosition(
                    positions[i].distanceMeters - previousPositions[i].distanceMeters,
                    positions[i].angle
                );
            }
            Twist2d twist = kinematics.toTwist2d(deltas);
            rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
            previousPositions = positions;
        }

        poseEstimator.update(rawGyroRotation, positions);

        for (Camera camera : cameras) {
            camera.updateInputs();
            for (VisionResult visionResult : camera.getResults()) {
                poseEstimator.addVisionMeasurement(visionResult.pose(), visionResult.time(), visionResult.stdDevs());
            }
        }
    }

    @AutoLogOutput(key="Drive/Speeds/Measured")
    public ChassisSpeeds getSpeeds() {
        return kinematics.toChassisSpeeds(getStates());
    }

    public void setSpeeds(ChassisSpeeds speeds) {
        Logger.recordOutput("Drive/Speeds/Setpoints", speeds);
        SwerveModuleState[] states;
        // if (
        //     speeds.vxMetersPerSecond == 0.0 &&
        //     speeds.vyMetersPerSecond == 0.0 &&
        //     speeds.omegaRadiansPerSecond == 0.0
        // ) {
        //     states = new SwerveModuleState[] {
        //         new SwerveModuleState(0.0, new Rotation2d(Math.PI / 4.0)),
        //         new SwerveModuleState(0.0, new Rotation2d(-Math.PI / 4.0)),
        //         new SwerveModuleState(0.0, new Rotation2d(-Math.PI / 4.0)),
        //         new SwerveModuleState(0.0, new Rotation2d(Math.PI / 4.0)),

        //     };
        // } else {
            states = kinematics.toSwerveModuleStates(speeds);
            SwerveDriveKinematics.desaturateWheelSpeeds(states, maxSpeed);
        // }

        setStates(states);
    }

    public void setSpeedsFieldOriented(ChassisSpeeds speeds) {
        setSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, rawGyroRotation));
    }

    @AutoLogOutput(key="Drive/Pose/Measured")
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void setPose(Pose2d pose) {
        Logger.recordOutput("Drive/Pose/Setpoint", pose);
        Pose2d currentPose = getPose();
        setSpeedsFieldOriented(
            new ChassisSpeeds(
                xController.calculate(currentPose.getX(), pose.getX()),
                yController.calculate(currentPose.getY(), pose.getY()),
                zController.calculate(currentPose.getRotation().getRadians(), pose.getRotation().getRadians())
            )
        );
    }

    @AutoLogOutput(key="Drive/States/Measured")
    public SwerveModuleState[] getStates() {
        SwerveModuleState[] states = new SwerveModuleState[modules.length];
        for (int i = 0; i < modules.length; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    public void setStates(SwerveModuleState[] states) {
        Logger.recordOutput("Drive/States/Setpoints", states);
        for (int i = 0; i < modules.length; i++) {
            states[i].optimize(modules[i].getRotation());
            modules[i].setState(states[i]);
        }
    }

    @AutoLogOutput(key="Drive/Positions/Measured")
    public SwerveModulePosition[] getPositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
        for (int i = 0; i < modules.length; i++) {
            positions[i] = modules[i].getPosition();
        }
        return positions;
    }
}
