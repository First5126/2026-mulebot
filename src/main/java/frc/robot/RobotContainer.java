// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import org.photonvision.PhotonCamera;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.constants.AprilTagLocalizationConstants;
import frc.robot.constants.WaypointConstants;
import frc.robot.constants.AprilTagLocalizationConstants.PhotonDetails;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandFactory;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.vision.AprilTagLocalization;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    PhotonDetails[] photonDetails = {
        AprilTagLocalizationConstants.camera1Details
    };
    public CommandFactory m_commandFactory = new CommandFactory(drivetrain);
    
    private AprilTagLocalization m_aprilTagLocalization =
        new AprilTagLocalization(
            drivetrain::getPose2d,
            drivetrain::resetPose,
            drivetrain::addVisionMeasurement,
            drivetrain,
            photonDetails,
            AprilTagLocalizationConstants.LIMELIGHT_DETAILS_RIGHT);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            drivetrain.gasPedalCommand(joystick::getRightTriggerAxis, joystick::getLeftTriggerAxis, joystick::getRightX, joystick::getLeftY, joystick::getLeftX));
        
        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.a().onTrue(m_aprilTagLocalization.setTrust(true));
        joystick.a().onFalse(m_aprilTagLocalization.setTrust(false));

        joystick.povUp().onTrue(drivetrain.goToPose(WaypointConstants.nearDepotPose).onlyWhile(null));
        joystick.povRight().onTrue(drivetrain.goToPose(WaypointConstants.nearHub));
        joystick.povDown().onTrue(drivetrain.goToPose(WaypointConstants.nearOutpost));
        joystick.povLeft().whileTrue(m_commandFactory.driveCircle());

        // Reset the field-centric heading on left bumper press.
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            // Then slowly drive forward (away from us) for 5 seconds.
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            .withTimeout(5.0),
            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> idle)
        );
    }
}
