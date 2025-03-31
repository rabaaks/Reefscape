package frc.robot.subsystems.roller;

import static frc.robot.subsystems.roller.RollerConstants.*;

import javax.security.auth.callback.ConfirmationCallback;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.roller.RollerIOInputsAutoLogged;

public class Roller extends SubsystemBase {
    private final RollerIO io;
    private final RollerIOInputsAutoLogged inputs = new RollerIOInputsAutoLogged();

    private final SimpleMotorFeedforward feedforward;

    public Roller(Config config, RollerIO io) {
        feedforward = new SimpleMotorFeedforward(config.s(), config.v(), config.a());

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
