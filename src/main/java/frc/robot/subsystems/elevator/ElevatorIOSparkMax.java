package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.FeedbackConfig;

import com.revrobotics.spark.config.SparkMaxConfig;

public class ElevatorIOSparkMax implements ElevatorIO {
    private final SparkMax leftMotor;
    private final SparkMax rightMotor;

    private final RelativeEncoder encoder;
    private final SparkClosedLoopController feedback;

    public ElevatorIOSparkMax(IOConfig ioConfig, RealConfig config) {
        leftMotor = new SparkMax(config.leftMotorId(), MotorType.kBrushless);
        rightMotor = new SparkMax(config.rightMotorId(), MotorType.kBrushless);

        SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig
            .inverted(false)
            .smartCurrentLimit(60)
            .idleMode(IdleMode.kCoast);
        motorConfig.encoder
            .positionConversionFactor(2.0 * Math.PI * ioConfig.radius() / ioConfig.gearing())
            .velocityConversionFactor(2.0 * Math.PI * ioConfig.radius() / ioConfig.gearing() / 60.0);
        leftMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motorConfig
            .follow(leftMotor, true);
        rightMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder = leftMotor.getEncoder();
        feedback = leftMotor.getClosedLoopController();
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.position = encoder.getPosition();
        inputs.velocity = encoder.getVelocity();
        inputs.voltages = new double[] {leftMotor.getAppliedOutput() * leftMotor.getBusVoltage(), rightMotor.getAppliedOutput() * rightMotor.getBusVoltage()};
        inputs.currents = new double[] {leftMotor.getOutputCurrent(), rightMotor.getOutputCurrent()};
    }

    @Override
    public void setPosition(double position, double ffVoltage) {
        feedback.setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0, ffVoltage);
    }

    @Override
    public void reset() {
        encoder.setPosition(0.0);
    }

    @Override
    public void setFeedback(FeedbackConfig feedbackConfig) {
        SparkMaxConfig pidConfig = new SparkMaxConfig();
        pidConfig.closedLoop
            .p(feedbackConfig.p())
            .i(feedbackConfig.i())
            .d(feedbackConfig.d());
        
        leftMotor.configure(pidConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        rightMotor.configure(pidConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }
}