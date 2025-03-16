// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.*;
import static frc.robot.subsystems.drive.DriveConstants.*;
import static frc.robot.subsystems.elevator.ElevatorConstants.*;
import static frc.robot.subsystems.roller.RollerConstants.*;

import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.VisionSystemSim;

import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.EncoderIO;
import frc.robot.subsystems.drive.EncoderIOReal;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOReal;
import frc.robot.subsystems.drive.Module;
import frc.robot.subsystems.drive.ModuleIOSparkMax;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIOSparkMax;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.roller.RollerIO;
import frc.robot.subsystems.roller.RollerIOSparkMax;
import frc.robot.subsystems.roller.RollerIOTalonSRX;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
    private final Drive drive;
    private final Elevator elevator;
    private final Roller shooter;

    private final CommandXboxController controller = new CommandXboxController(driverControllerPort);

    public RobotContainer() {
        switch (currentMode) {
            case REAL:
                // drive = new Drive(
                //     new GyroIOReal(gyroId),
                //     new Module[] {
                //         new Module(new ModuleIOSparkMax(frontLeftDriveId, frontLeftTurnId), new EncoderIOReal(frontLeftEncoderId, frontLeftOffset), 0),
                //         new Module(new ModuleIOSparkMax(frontRightDriveId, frontRightTurnId), new EncoderIOReal(frontRightEncoderId, frontRightOffset), 1),
                //         new Module(new ModuleIOSparkMax(backLeftDriveId, backLeftTurnId), new EncoderIOReal(backLeftEncoderId, backLeftOffset), 2),
                //         new Module(new ModuleIOSparkMax(backRightDriveId, backRightTurnId), new EncoderIOReal(backRightEncoderId, backRightOffset), 3)
                //     }
                // );
                drive = new Drive(
                    new GyroIO() {},
                    new Module[] {
                        new Module(new ModuleIOSim(), new EncoderIO() {}, 0),
                        new Module(new ModuleIOSim(), new EncoderIO() {}, 1),
                        new Module(new ModuleIOSim(), new EncoderIO() {}, 2),
                        new Module(new ModuleIOSim(), new EncoderIO() {}, 3)
                    }
                );
                elevator = new Elevator(new ElevatorIOSparkMax(leftMotorId, rightMotorId));
                shooter = new Roller(new RollerIOSparkMax(flywheelId));
                break;
            default:
            case SIM:
            case REPLAY:

                drive = new Drive(
                    new GyroIO() {},
                    new Module[] {
                        new Module(new ModuleIOSim(), new EncoderIO() {}, 0),
                        new Module(new ModuleIOSim(), new EncoderIO() {}, 1),
                        new Module(new ModuleIOSim(), new EncoderIO() {}, 2),
                        new Module(new ModuleIOSim(), new EncoderIO() {}, 3)
                    }
                );
                elevator = new Elevator(new ElevatorIOSim());
                shooter = new Roller(new RollerIO() {});
                break;
        }

        configureBindings();
    }

    private void configureBindings() {
        drive.setDefaultCommand(
            new RunCommand(
                () -> drive.setSpeedsFieldOriented(
                    new ChassisSpeeds(
                        MathUtil.applyDeadband(-controller.getLeftY(), 0.15) * driveSpeed, 
                        MathUtil.applyDeadband(-controller.getLeftX(), 0.15) * driveSpeed, 
                        MathUtil.applyDeadband(-controller.getRightX(), 0.15) * turnSpeed
                    )
                ),
                drive
            )
        );

        // controller.povDown().onTrue(new RunCommand(() -> elevator.setPosition(0.0), elevator));
        // controller.povLeft().onTrue(new RunCommand(() -> elevator.setPosition(0.4), elevator));
        // controller.povRight().onTrue(new RunCommand(() -> elevator.setPosition(0.8), elevator));
        // controller.povUp().onTrue(new RunCommand(() -> elevator.setPosition(1.0), elevator));

        // controller.a().whileTrue(new RunCommand(() -> shooter.setVelocity(1000)));

        elevator.setDefaultCommand(
            new RunCommand(
                () -> {
                    elevator.setPosition(
                        controller.povUp().getAsBoolean() ? 1.0 : (controller.povLeft().getAsBoolean() ? 0.6 : (controller.povRight().getAsBoolean() ? 0.4 : 0.1))
                    );
                },
                elevator
            )
        );

        shooter.setDefaultCommand(
            new RunCommand(
                () -> {
                    shooter.setVelocity(controller.getRightTriggerAxis() * 12.0);
                },
                shooter
            )
        );

        controller.a().whileTrue(elevator.sysIdRoutine());
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
