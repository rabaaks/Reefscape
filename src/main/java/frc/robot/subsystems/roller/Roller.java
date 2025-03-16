package frc.robot.subsystems.roller;

import static frc.robot.subsystems.roller.RollerConstants.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.ShooterIOInputsAutoLogged;

public class Roller extends SubsystemBase {
    private final RollerIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(s, v, a);

    public Roller(RollerIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
    }

    public void setVelocity(double velocity) {
        Logger.recordOutput("Shooter/VelocitySetpoint", velocity);
        io.setVoltage(feedforward.calculate(velocity));
    }
}
