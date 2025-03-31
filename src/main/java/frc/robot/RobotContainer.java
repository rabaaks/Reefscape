// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.VisionSystemSim;

import frc.robot.subsystems.drive.Camera;
import frc.robot.subsystems.drive.CameraIOReal;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.EncoderIO;
import frc.robot.subsystems.drive.EncoderIOReal;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOReal;
import frc.robot.subsystems.drive.Module;
import frc.robot.subsystems.drive.ModuleIOSparkMax;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorIOSparkMax;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.roller.RollerConstants;
import frc.robot.subsystems.roller.RollerIO;
import frc.robot.subsystems.roller.RollerIOTalonSRX;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
    private final Drive drive;
    private final Elevator elevator;
    private final Roller roller;

    private final CommandXboxController controller = new CommandXboxController(Constants.driverControllerPort);

    public RobotContainer() {
        switch (Constants.currentMode) {
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
                    DriveConstants.protoConfig,
                    new GyroIOReal(DriveConstants.protoGyroConfig),
                    new Module[] {
                        new Module(new ModuleIOSparkMax(DriveConstants.protoModuleIOConfig, DriveConstants.protoFrontLeftModuleRealConfig), new EncoderIOReal(DriveConstants.protoFrontLeftEncoderRealConfig), 0),
                        new Module(new ModuleIOSparkMax(DriveConstants.protoModuleIOConfig, DriveConstants.protoFrontRightModuleRealConfig), new EncoderIOReal(DriveConstants.protoFrontRightEncoderRealConfig), 1),
                        new Module(new ModuleIOSparkMax(DriveConstants.protoModuleIOConfig, DriveConstants.protoBackLeftModuleRealConfig), new EncoderIOReal(DriveConstants.protoBackLeftEncoderRealConfig), 2),
                        new Module(new ModuleIOSparkMax(DriveConstants.protoModuleIOConfig, DriveConstants.protoBackRightModuleRealConfig), new EncoderIOReal(DriveConstants.protoBackRightEncoderRealConfig), 3)
                    },
                    new Camera[] {}
                );
                elevator = new Elevator(ElevatorConstants.protoConfig, new ElevatorIOSparkMax(ElevatorConstants.protoIOConfig, ElevatorConstants.protoRealConfig));
                roller = new Roller(RollerConstants.protoConfig, new RollerIOTalonSRX(RollerConstants.protoRealConfig));
                break;
            default:
            case SIM:
            case REPLAY:
                drive = new Drive(
                    DriveConstants.protoConfig,
                    new GyroIO() {},
                    new Module[] {
                        new Module(new ModuleIOSim(DriveConstants.protoModuleIOConfig, DriveConstants.protoModuleSimConfig), new EncoderIO() {}, 0),
                        new Module(new ModuleIOSim(DriveConstants.protoModuleIOConfig, DriveConstants.protoModuleSimConfig), new EncoderIO() {}, 1)/*,*/
                        // new Module(new ModuleIOSim(DriveConstants.protoModuleIOConfig, DriveConstants.protoModuleSimConfig), new EncoderIO() {}, 2),
                        // new Module(new ModuleIOSim(DriveConstants.protoModuleIOConfig, DriveConstants.protoModuleSimConfig), new EncoderIO() {}, 3)
                    },
                    new Camera[] {
                        new Camera(new CameraIOReal(DriveConstants.protoCameraIOConfig, DriveConstants.protoCameraRealConfig), 0)
                    }
                );
                elevator = new Elevator(ElevatorConstants.protoConfig, new ElevatorIOSim(ElevatorConstants.protoIOConfig, ElevatorConstants.protoSimConfig));
                roller = new Roller(RollerConstants.protoConfig, new RollerIO() {});
                break;
        }

        configureBindings();
    }

    private void configureBindings() {
        drive.setDefaultCommand(
            new RunCommand(
                () -> drive.setSpeedsFieldOriented(
                    new ChassisSpeeds(
                        MathUtil.applyDeadband(-controller.getLeftY(), 0.15) * 3.0, 
                        MathUtil.applyDeadband(-controller.getLeftX(), 0.15) * 3.0, 
                        MathUtil.applyDeadband(-controller.getRightX(), 0.15) * 4.5
                    )
                ),
                drive
            )
        );
        controller.b().whileTrue(new RunCommand(() -> drive.setPose(new Pose2d(1, 1, new Rotation2d())), drive));

        elevator.setDefaultCommand(new RunCommand(() -> elevator.setPosition(0.0), elevator));
        controller.povLeft().whileTrue(new RunCommand(() -> elevator.setPosition(0.4), elevator));
        controller.povRight().whileTrue(new RunCommand(() -> elevator.setPosition(0.8), elevator));
        controller.povUp().whileTrue(new RunCommand(() -> elevator.setPosition(1.0), elevator));

        roller.setDefaultCommand(
            new RunCommand(
                () -> {
                    roller.setVelocity(controller.getRightTriggerAxis() * 12.0);
                },
                roller
            )
        );

        controller.a().whileTrue(elevator.sysIdRoutine(0.9, 0.1));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
