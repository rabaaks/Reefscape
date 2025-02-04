package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ModuleIOSim implements ModuleIO {
    private DCMotorSim driveSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(driveMotor, driveMOI, driveGearing),
        driveMotor
    );

    private DCMotorSim turnSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(turnMotor, turnMOI, turnGearing),
        turnMotor
    );

    private PIDController driveFeedback = new PIDController(driveP, driveI, driveD);
    private double driveFeedforward = 0.0;

    private PIDController turnFeedback = new PIDController(turnP, turnI, turnD);
    private double turnFeedforward = 0.0;

    public ModuleIOSim() {
        turnFeedback.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        double driveVelocity = driveSim.getAngularVelocityRadPerSec() * wheelRadius;
        double turnPosition = turnSim.getAngularPositionRad();
        double driveVoltage = driveFeedforward + driveFeedback.calculate(driveVelocity);
        double turnVoltage = turnFeedforward + turnFeedback.calculate(turnPosition);

        driveSim.setInputVoltage(MathUtil.clamp(driveVoltage, -12.0, 12.0));
        driveSim.update(0.02);

        turnSim.setInputVoltage(MathUtil.clamp(turnVoltage, -12.0, 12.0));
        turnSim.update(0.02);

        inputs.driveVelocity = driveVelocity;
        inputs.turnVelocity = turnSim.getAngularVelocityRadPerSec();
        inputs.drivePosition = driveSim.getAngularPositionRad() * wheelRadius; 
        inputs.turnPosition = turnPosition;

        inputs.driveVoltage = driveVoltage;
        inputs.turnVoltage = turnVoltage;
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
}