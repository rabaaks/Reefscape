package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.math.controller.ElevatorFeedforward;
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

    private final TrapezoidProfile profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(maxProfileVelocity, maxProfileAcceleration));
    private TrapezoidProfile.State profileState = new TrapezoidProfile.State(0, 0);
    private TrapezoidProfile.State futureProfileState = new TrapezoidProfile.State(0, 0);

    private ElevatorFeedforward feedforward = new ElevatorFeedforward(s, g, v, a);

    private final SysIdRoutine routine;

    public Elevator(ElevatorIO io) {
        this.io = io;

        routine = new SysIdRoutine(
        new SysIdRoutine.Config(
            Velocity.ofRelativeUnits(sysIdRampUp, Units.Volts.per(Units.Seconds)), 
            Voltage.ofRelativeUnits(sysIdStep, Units.Volts), 
            Time.ofRelativeUnits(sysIdTimeout, Units.Seconds)
        ), 
        new SysIdRoutine.Mechanism(
            voltage -> io.setPosition(0, voltage.magnitude()),
            log -> {
                log.motor("elevator")
                    .voltage(Voltage.ofRelativeUnits(inputs.voltages[0], Units.Volts))
                    .linearPosition(Distance.ofRelativeUnits(inputs.position, Units.Meters))
                    .linearVelocity(LinearVelocity.ofRelativeUnits(inputs.velocity, Units.MetersPerSecond));
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

    @AutoLogOutput(key="Elevator/PositionMeasured")
    public double getPosition() {
        return inputs.position;
    }

    public void setPosition(double position) {
        futureProfileState = profile.calculate(0.02, profileState, new TrapezoidProfile.State(position, 0.0));
        Logger.recordOutput("Elevator/PositionSetpoint", profileState.position);
        Logger.recordOutput("Elevator/VelocitySetpoint", profileState.velocity);
        io.setPosition(profileState.position, feedforward.calculateWithVelocities(profileState.velocity, futureProfileState.velocity));
        profileState = futureProfileState;
    }

    @AutoLogOutput(key="Elevator/VelocityMeasured")
    public double getVelocity() {
        return inputs.velocity;
    }

    public void reset() {
        io.reset();
    }

    public Command sysIdRoutine() {
        return Commands.sequence(
            routine.quasistatic(SysIdRoutine.Direction.kForward).until(() -> inputs.position > sysIdMaxPosition),
            routine.quasistatic(SysIdRoutine.Direction.kReverse).until(() -> inputs.position < sysIdMinPosition),
            routine.dynamic(SysIdRoutine.Direction.kForward).until(() -> inputs.position > sysIdMaxPosition),
            routine.dynamic(SysIdRoutine.Direction.kReverse).until(() -> inputs.position < sysIdMinPosition)
        );
    }
}
