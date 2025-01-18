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

import frc.robot.subsystems.drive.ModuleIO.ModuleIOInputs;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorIOInputs;

public class ModuleIOReal implements ModuleIO {
    private SparkMax driveMotor;
    private SparkMax turnMotor;

    private final RelativeEncoder driveEncoder = driveMotor.getEncoder();
    private final RelativeEncoder turnEncoder = turnMotor.getEncoder();

    private final SparkClosedLoopController driveFeedback = driveMotor.getClosedLoopController();
    private final SparkClosedLoopController turnFeedback = turnMotor.getClosedLoopController();

    private double driveFeedforward = 0.0;
    private double turnFeedforward = 0.0;

    public ModuleIOReal(int driveId, int turnId) {
        driveMotor = new SparkMax(driveId, MotorType.kBrushless);
        turnMotor = new SparkMax(turnId, MotorType.kBrushless);

        SparkMaxConfig driveConfig = new SparkMaxConfig();
        SparkMaxConfig turnConfig = new SparkMaxConfig();

        driveConfig
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kCoast);
        driveConfig.encoder
            .positionConversionFactor(drivePositionConversionFactor)
            .velocityConversionFactor(driveVelocityConversionFactor);
        driveConfig.closedLoop
            .p(driveP)
            .i(driveI)
            .d(driveD);

        turnConfig
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kCoast);
        turnConfig.encoder
            .positionConversionFactor(turnPositionConversionFactor)
            .velocityConversionFactor(turnVelocityConversionFactor);
        turnConfig.closedLoop
            .p(turnP)
            .i(turnI)
            .d(turnD)
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
}
