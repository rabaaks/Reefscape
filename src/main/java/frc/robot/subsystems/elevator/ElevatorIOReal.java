package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

public class ElevatorIOReal implements ElevatorIO {
    private SparkMax leftMotor = new SparkMax(leftMotorId, MotorType.kBrushless);
    private SparkMax rightMotor = new SparkMax(rightMotorId, MotorType.kBrushless);

    private RelativeEncoder encoder = leftMotor.getEncoder();
    private SparkClosedLoopController feedback = leftMotor.getClosedLoopController();
    private double feedforward = 0.0;

    public ElevatorIOReal() {
        SparkMaxConfig config = new SparkMaxConfig();
        config
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kCoast);
        config.encoder
            .positionConversionFactor(positionConversionFactor)
            .velocityConversionFactor(velocityConversionFactor);
        config.closedLoop
            .p(p)
            .i(i)
            .d(d);
        leftMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        config
            .follow(leftMotor, true);
        rightMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.position = encoder.getPosition();
        inputs.velocity = encoder.getVelocity();
        inputs.voltages = new double[] {leftMotor.getBusVoltage(), rightMotor.getBusVoltage()};
        inputs.currents = new double[] {leftMotor.getOutputCurrent(), rightMotor.getOutputCurrent()};
    }

    @Override
    public void setPosition(double position, double ffVoltage) {
        feedback.setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0, ffVoltage);
    }
}