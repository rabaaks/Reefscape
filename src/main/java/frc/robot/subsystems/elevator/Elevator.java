package frc.robot.subsystems.elevator;

import static frc.robot.Constants.*;
import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkInput;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.epilogue.Logged;
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
import frc.robot.Constants.FeedbackConfig;
import frc.robot.subsystems.elevator.ElevatorConstants.Config;

public class Elevator extends SubsystemBase {
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    private ExponentialProfile profile;
    private ExponentialProfile.State profileState = new ExponentialProfile.State(0.0, 0.0);
    private ExponentialProfile.State futureProfileState = new ExponentialProfile.State(0.0, 0.0);

    private ElevatorFeedforward feedforward;

    private final SysIdRoutine routine;

    private final LoggedNetworkNumber p = new LoggedNetworkNumber("/Tuning/Elevator/P");
    private final LoggedNetworkNumber i = new LoggedNetworkNumber("/Tuning/Elevator/I");
    private final LoggedNetworkNumber d = new LoggedNetworkNumber("/Tuning/Elevator/D");
    private final LoggedNetworkNumber s = new LoggedNetworkNumber("/Tuning/Elevator/S");
    private final LoggedNetworkNumber g = new LoggedNetworkNumber("/Tuning/Elevator/G");
    private final LoggedNetworkNumber v = new LoggedNetworkNumber("/Tuning/Elevator/V");
    private final LoggedNetworkNumber a = new LoggedNetworkNumber("/Tuning/Elevator/A");

    double maxProfileVoltage;

    public Elevator(Config config, ElevatorIO io) {
        this.io = io;

        p.setDefault(config.feedback().p());
        i.setDefault(config.feedback().i());
        d.setDefault(config.feedback().d());
        s.setDefault(config.feedforward().s());
        g.setDefault(config.feedforward().g());
        v.setDefault(config.feedforward().v());
        a.setDefault(config.feedforward().a());

        maxProfileVoltage = config.maxProfileVoltage();

        periodic();

        profileState.position = inputs.position;

        routine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Units.Volts.per(Units.Seconds).of(sysIdRampUp),
                Units.Volts.of(sysIdStep), 
                Units.Seconds.of(sysIdTimeout)
            ), 
            new SysIdRoutine.Mechanism(
                voltage -> io.setPosition(0, voltage.magnitude()),
                log -> {
                    log.motor("elevator")
                        .voltage(Units.Volts.of(inputs.voltages[0]))
                        .linearPosition(Units.Meters.of(inputs.position))
                        .linearVelocity(Units.MetersPerSecond.of(inputs.velocity));
                },
                this
            )
        );
    }

    @Override
    public void periodic() {
        io.setFeedback(new FeedbackConfig(p.get(), i.get(), d.get()));
        feedforward = new ElevatorFeedforward(s.get(), g.get(), v.get());
        profile = new ExponentialProfile(ExponentialProfile.Constraints.fromCharacteristics(maxProfileVoltage - s.get() - g.get(), v.get(), a.get()));

        
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
