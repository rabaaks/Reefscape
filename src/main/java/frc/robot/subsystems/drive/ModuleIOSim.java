package frc.robot.subsystems.drive;

import static frc.robot.Constants.*;
import static frc.robot.subsystems.drive.DriveConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ModuleIOSim implements ModuleIO {
    private DCMotorSim driveSim;
    private DCMotorSim turnSim;

    private PIDController driveFeedback;
    private double driveFeedforward = 0.0;

    private PIDController turnFeedback;
    private double turnFeedforward = 0.0;

    private double wheelRadius;

    public ModuleIOSim(ModuleIOConfig ioConfig, ModuleSimConfig config) {
        driveSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(config.driveMotor(), config.driveMOI(), ioConfig.driveGearing()),
            config.driveMotor()
        );
        turnSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(config.turnMotor(), config.turnMOI(), ioConfig.turnGearing()),
            config.turnMotor()
        );

        driveFeedback = new PIDController(0.0, 0.0, 0.0);
        turnFeedback = new PIDController(0.0, 0.0, 0.0);

        wheelRadius = ioConfig.wheelRadius();

        turnFeedback.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        double driveVelocity = driveSim.getAngularVelocityRadPerSec() * wheelRadius;
        double turnPosition = turnSim.getAngularPositionRad();
        double driveOutput = driveFeedforward + driveFeedback.calculate(driveVelocity);
        double turnOutput = turnFeedforward + turnFeedback.calculate(turnPosition);

        driveSim.setInputVoltage(MathUtil.clamp(driveOutput, -1.0, 1.0) * 12.0);
        driveSim.update(0.02);

        turnSim.setInputVoltage(MathUtil.clamp(turnOutput, -1.0, 1.0) * 12.0);
        turnSim.update(0.02);

        inputs.driveVelocity = driveVelocity;
        inputs.turnVelocity = turnSim.getAngularVelocityRadPerSec();
        inputs.drivePosition = driveSim.getAngularPositionRad() * wheelRadius; 
        inputs.turnPosition = turnPosition;

        inputs.driveVoltage = driveOutput;
        inputs.turnVoltage = turnOutput;
        inputs.driveCurrent = driveSim.getCurrentDrawAmps();
        inputs.turnCurrent = turnSim.getCurrentDrawAmps();
    }

    @Override
    public void setDriveVelocity(double velocity, double ffVoltage) {
        driveFeedforward = ffVoltage;
        driveFeedback.setSetpoint(velocity);
    }

    @Override
    public void setTurnPosition(double position, double ffVoltage) {
        turnFeedforward = ffVoltage;
        turnFeedback.setSetpoint(position);
    }

    @Override
    public void setDriveFeedback(FeedbackConfig config) {
        driveFeedback.setP(config.p());
        driveFeedback.setI(config.i());
        driveFeedback.setD(config.d());
    }

    @Override
    public void setTurnFeedback(FeedbackConfig config) {
        turnFeedback.setP(config.p());
        turnFeedback.setI(config.i());
        turnFeedback.setD(config.d());
    }
}