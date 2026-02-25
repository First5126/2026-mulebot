// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.FMS.ShiftData;
import frc.robot.FMS.Zones;
import frc.robot.constants.AprilTagLocalizationConstants;
import frc.robot.constants.AprilTagLocalizationConstants.PhotonDetails;
import frc.robot.controller.Driver;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandFactory;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LEDLights;
import frc.robot.subsystems.ShootingMechanism;
import frc.robot.subsystems.Turret;
import frc.robot.vision.AprilTagLocalization;

public class RobotContainer {
  private double MaxSpeed =
      1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate =
      RotationsPerSecond.of(0.75)
          .in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(
              DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  public final CommandSwerveDrivetrain m_drivetrain = TunerConstants.createDrivetrain();

  // subsystems
  private final Turret m_turret = new Turret();
  private final Zones m_zone = new Zones(m_drivetrain::getPose2d);
  private final ShiftData m_shiftData = new ShiftData();
  private final LEDLights m_ledLights = new LEDLights(m_shiftData);

  PhotonDetails[] photonDetails = {
    // AprilTagLocalizationConstants.camera1Details
  };
  public ShootingMechanism m_shootingMechanism =
      new ShootingMechanism(m_turret, m_drivetrain, m_zone);
  public CommandFactory m_commandFactory =
      new CommandFactory(m_drivetrain, m_turret, m_zone, m_shootingMechanism);

  private AprilTagLocalization m_aprilTagLocalization =
      new AprilTagLocalization(
          m_drivetrain::getPose2d,
          m_drivetrain::resetPose,
          m_drivetrain::addVisionMeasurement,
          m_drivetrain,
          photonDetails,
          AprilTagLocalizationConstants.LIMELIGHT_DETAILS_RIGHTG);

  private final Telemetry logger = new Telemetry(MaxSpeed);

  public RobotContainer() {
    configureBindings();

    //m_turret.setDefaultCommand(m_turret.rotateToPosition(m_shootingMechanism::getShootingSolution));
    // TODO: add hood default command
  }

  private void configureBindings() {
    Driver.init(m_drivetrain, m_aprilTagLocalization, m_commandFactory, m_zone, m_ledLights)
        .configureBindings();

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled()
        .whileTrue(m_drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    m_drivetrain.registerTelemetry(logger::telemeterize);
  }

  public Command getAutonomousCommand() {
    // Simple drive forward auton
    final var idle = new SwerveRequest.Idle();
    return Commands.sequence(
        // Reset our field centric heading to match the robot
        // facing away from our alliance station wall (0 deg).
        m_drivetrain.runOnce(() -> m_drivetrain.seedFieldCentric(Rotation2d.kZero)),
        // Then slowly drive forward (away from us) for 5 seconds.s
        // Finally idle for the rest of auton
        m_drivetrain.applyRequest(() -> idle));
  }
}
