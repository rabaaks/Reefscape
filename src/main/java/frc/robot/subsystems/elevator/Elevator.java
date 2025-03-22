package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Elevator extends SubsystemBase {
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    private final ExponentialProfile profile;
    private ExponentialProfile.State profileState = new ExponentialProfile.State(0.0, 0.0);
    private ExponentialProfile.State futureProfileState = new ExponentialProfile.State(0.0, 0.0);

    private ElevatorFeedforward feedforward;

    private final SysIdRoutine routine;

    public Elevator(Config config, ElevatorIO io) {
        this.io = io;

        profile = new ExponentialProfile(ExponentialProfile.Constraints.fromCharacteristics(config.maxProfileVoltage() - config.s() - config.g(), config.v(), config.a()));
        feedforward = new ElevatorFeedforward(config.s(), config.g(), config.v());

        periodic();

        profileState.position = inputs.position;

        routine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Units.Volts.per(Units.Seconds).of(sysIdRampUp),
                Units.Volts.of(sysIdStep), 
                Units.Seconds.of(sysIdTimeout)
            ), 
            new SysIdRoutine.Mechanism(
                voltage -> io.setPosition(0, voltage.magnitude() / 12.0),
                log -> {
                    log.motor("elevator")
                        .voltage(Units.Volts.of(inputs.outputs[0] * 12.0))
                        .linearPosition(Units.Meters.of(inputs.position))
                        .linearVelocity(Units.MetersPerSecond.of(inputs.velocity));
                },
                this
            )
        );
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
    }

    @AutoLogOutput(key="Elevator/Position/Measured")
    public double getPosition() {
        return inputs.position;
    }

    public void setPosition(double position) {
        futureProfileState = profile.calculate(0.02, profileState, new ExponentialProfile.State(position, 0.0));
        Logger.recordOutput("Elevator/Position/Setpoint", profileState.position);
        Logger.recordOutput("Elevator/Velocity/Setpoint", profileState.velocity);
        double feedforwardValue = feedforward.calculateWithVelocities(profileState.velocity, futureProfileState.velocity);
        Logger.recordOutput("Elevator/Feedforward", feedforwardValue);
        io.setPosition(futureProfileState.position, feedforwardValue);
        profileState = futureProfileState;
    }

    @AutoLogOutput(key="Elevator/Velocity/Measured")
    public double getVelocity() {
        return inputs.velocity;
    }

    public void reset() {
        io.reset();
    }

    public Command sysIdRoutine(double max, double min) {
        return Commands.sequence(
            routine.quasistatic(SysIdRoutine.Direction.kForward).until(() -> inputs.position > max),
            routine.quasistatic(SysIdRoutine.Direction.kReverse).until(() -> inputs.position < min),
            routine.dynamic(SysIdRoutine.Direction.kForward).until(() -> inputs.position > max),
            routine.dynamic(SysIdRoutine.Direction.kReverse).until(() -> inputs.position < min)
        );
    }
}
