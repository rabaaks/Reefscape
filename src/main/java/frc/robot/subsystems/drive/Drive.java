package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Drive extends SubsystemBase {
    private final Module[] modules;

    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    private Rotation2d rawGyroRotation = new Rotation2d();
    
    private SwerveModulePosition[] previousPositions = new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
    };

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        new Translation2d[] {
            new Translation2d(halfWidth, halfWidth),
            new Translation2d(halfWidth, -halfWidth),
            new Translation2d(-halfWidth, halfWidth),
            new Translation2d(-halfWidth, -halfWidth)
        }
    );
    private final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(kinematics, new Rotation2d(), previousPositions, new Pose2d());

    public Drive(GyroIO gyroIO, Module[] modules) {
        this.gyroIO = gyroIO;
        this.modules = modules;
    }

    @Override
    public void periodic() {
        for (Module module : modules) {
            module.updateInputs();
        }

        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);

        SwerveModulePosition[] positions = getPositions();
        if (gyroInputs.connected) {
            rawGyroRotation = new Rotation2d(gyroInputs.yawPosition);
        } else {
            SwerveModulePosition[] deltas = new SwerveModulePosition[4];
            for (int i = 0; i < 4; i++) {
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
    }

    @AutoLogOutput(key="Drive/Speeds/Measured")
    public ChassisSpeeds getSpeeds() {
        return kinematics.toChassisSpeeds(getStates());
    }

    public void setSpeeds(ChassisSpeeds speeds) {
        Logger.recordOutput("Drive/Speeds/Setpoints", speeds);
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, driveSpeed);

        setStates(states);
    }

    public void setSpeedsFieldOriented(ChassisSpeeds speeds) {
        setSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, rawGyroRotation));
    }

    @AutoLogOutput(key="Drive/Pose/Measured")
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    @AutoLogOutput(key="Drive/States/Measured")
    public SwerveModuleState[] getStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    public void setStates(SwerveModuleState[] states) {
        Logger.recordOutput("Drive/States/Setpoints", states);
        for (int i = 0; i < 4; i++) {
            modules[i].setState(states[i]);
        }
    }

    @AutoLogOutput(key="Drive/Positions/Measured")
    public SwerveModulePosition[] getPositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            positions[i] = modules[i].getPosition();
        }
        return positions;
    }
}
