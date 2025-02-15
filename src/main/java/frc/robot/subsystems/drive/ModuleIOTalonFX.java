package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.units.VelocityUnit;

public class ModuleIOTalonFX implements ModuleIO {
    private TalonFX driveMotor;
    private TalonFX turnMotor;

    public ModuleIOTalonFX(int driveId, int turnId) {
        driveMotor = new TalonFX(driveId);
        turnMotor = new TalonFX(turnId);

        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        TalonFXConfiguration turnConfig = new TalonFXConfiguration();

        driveConfig.MotorOutput
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Coast);

        driveConfig.CurrentLimits
            .withStatorCurrentLimit(40);

        driveConfig.Feedback
            .withSensorToMechanismRatio(1 / driveGearing);
        
        driveConfig.Slot0
            .withKP(driveP)
            .withKI(driveI)
            .withKD(driveD);

        turnConfig.MotorOutput
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Coast);

        turnConfig.CurrentLimits
            .withStatorCurrentLimit(40);
        
        turnConfig.Feedback
            .withSensorToMechanismRatio(1 / turnGearing);
        
        turnConfig.ClosedLoopGeneral
            .withContinuousWrap(true);

        turnConfig.Slot0
            .withKP(turnP)
            .withKI(turnI)
            .withKD(turnD);
        
        driveMotor.getConfigurator().apply(driveConfig);
        turnMotor.getConfigurator().apply(turnConfig);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        inputs.drivePosition = driveMotor.getPosition().getValueAsDouble() * 2.0 * Math.PI * wheelRadius;
        inputs.turnPosition = turnMotor.getPosition().getValueAsDouble() * 2.0 * Math.PI;
        inputs.driveVelocity = driveMotor.getVelocity().getValueAsDouble() * 2.0 * Math.PI * wheelRadius;
        inputs.turnVelocity = turnMotor.getVelocity().getValueAsDouble() * 2.0 * Math.PI;

        inputs.driveVoltage = 0.0;
        inputs.turnVoltage = 0.0;
        inputs.driveCurrent = driveMotor.getStatorCurrent().getValueAsDouble();
        inputs.turnCurrent = turnMotor.getStatorCurrent().getValueAsDouble();
    }

    @Override
    public void setDriveVelocity(double velocity, double ffVoltage) {
        VelocityVoltage request = new VelocityVoltage(0).withSlot(0);
        driveMotor.setControl(request.withVelocity(velocity / (2.0 * Math.PI * wheelRadius)).withFeedForward(ffVoltage));
    }

    @Override
    public void setTurnPosition(double position, double ffVoltage) {
        PositionVoltage request = new PositionVoltage(0).withSlot(0);
        turnMotor.setControl(request.withPosition(position / (2.0 * Math.PI)).withFeedForward(ffVoltage));
    }

    @Override
    public void resetPosition(double position) {
        turnMotor.setPosition(position);
    }
}
