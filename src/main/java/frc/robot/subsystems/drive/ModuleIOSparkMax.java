package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.FeedbackConfig;
import frc.robot.subsystems.drive.DriveConstants.ModuleIOConfig;
import frc.robot.subsystems.drive.DriveConstants.ModuleRealConfig;

public class ModuleIOSparkMax implements ModuleIO {
    private SparkMax driveMotor;
    private SparkMax turnMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turnEncoder;

    private final SparkClosedLoopController driveFeedback;
    private final SparkClosedLoopController turnFeedback;

    public ModuleIOSparkMax(ModuleIOConfig ioConfig, ModuleRealConfig config) {
        driveMotor = new SparkMax(config.driveMotorId(), MotorType.kBrushless);
        turnMotor = new SparkMax(config.turnMotorId(), MotorType.kBrushless);

        driveEncoder = driveMotor.getEncoder();
        turnEncoder = turnMotor.getEncoder();

        driveFeedback = driveMotor.getClosedLoopController();
        turnFeedback = turnMotor.getClosedLoopController();

        SparkMaxConfig driveConfig = new SparkMaxConfig();
        SparkMaxConfig turnConfig = new SparkMaxConfig();

        driveConfig
            .inverted(true)
            .smartCurrentLimit(60)
            .idleMode(IdleMode.kCoast);
        driveConfig.encoder
            .positionConversionFactor(2.0 * Math.PI * ioConfig.wheelRadius() / ioConfig.driveGearing())
            .velocityConversionFactor(2.0 * Math.PI * ioConfig.wheelRadius() / ioConfig.driveGearing() / 60.0);

        turnConfig
            .inverted(false)
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kCoast);
        turnConfig.encoder
            .positionConversionFactor(2.0 * Math.PI / ioConfig.turnGearing())
            .velocityConversionFactor(2.0 * Math.PI / ioConfig.turnGearing() / 60.0);
        turnConfig.closedLoop
            .positionWrappingInputRange(-Math.PI, Math.PI)
            .positionWrappingEnabled(true);
        
        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        turnMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        inputs.drivePosition = driveEncoder.getPosition();
        inputs.turnPosition = turnEncoder.getPosition();
        inputs.driveVelocity = driveEncoder.getVelocity();
        inputs.turnVelocity = turnEncoder.getVelocity();

        inputs.driveVoltage = driveMotor.getAppliedOutput() * driveMotor.getBusVoltage();
        inputs.turnVoltage = turnMotor.getAppliedOutput() * turnMotor.getBusVoltage();
        inputs.driveCurrent = driveMotor.getOutputCurrent();
        inputs.turnCurrent = turnMotor.getOutputCurrent();
    }

    @Override
    public void setDriveVelocity(double velocity, double ffVoltage) {
        driveFeedback.setReference(velocity, ControlType.kVelocity, ClosedLoopSlot.kSlot0, ffVoltage);
    }

    @Override
    public void setTurnPosition(double position, double ffVoltage) {
        turnFeedback.setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0, ffVoltage);
    }

    @Override
    public void resetPosition(double position) {
        turnEncoder.setPosition(position);
    }

    @Override
    public void setDriveFeedback(FeedbackConfig config) {
        SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig.closedLoop
            .p(config.p())
            .i(config.i())
            .d(config.d());
        
        driveMotor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setTurnFeedback(FeedbackConfig config) {
        SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig.closedLoop
            .p(config.p())
            .i(config.i())
            .d(config.d());
        
        turnMotor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }
}
