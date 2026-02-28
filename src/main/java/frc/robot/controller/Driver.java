package frc.robot.controller;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FMS.ShiftData;
import frc.robot.FMS.Zones;
import frc.robot.constants.ControllerConstants;
import frc.robot.subsystems.CommandFactory;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDLights;
import frc.robot.subsystems.Turret;
import frc.robot.vision.AprilTagLocalization;
import lombok.Getter;
import lombok.Setter;

public class Driver extends CustomXboxController implements Controller {
  // Singleton instance
  private static Driver INSTANCE;

  @Getter @Setter private CommandSwerveDrivetrain drivetrain;
  @Getter @Setter private AprilTagLocalization aprilTagLocalization;
  @Getter @Setter private CommandFactory commandFactory;
  @Getter @Setter private Intake intake = new Intake();
  @Getter @Setter private Turret turret = new Turret();
  @Getter @Setter private LEDLights ledLights;
  @Getter @Setter private Zones zones;

  private final SwerveRequest.SwerveDriveBrake BRAKE = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt POINT = new SwerveRequest.PointWheelsAt();

  // Private constructor to prevent instantiation from outside
  private Driver() {
    super(ControllerConstants.DRIVER_CONTROLLER_PORT);
  }

  // Public method to access the single instance
  public static Driver getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new Driver();
    }
    return INSTANCE;
  }

  public static Driver init(
      CommandSwerveDrivetrain drivetrain,
      AprilTagLocalization aprilTagLocalization,
      CommandFactory commandFactory,
      Zones zones,
      LEDLights ledLights) {
    Driver driver = getInstance();
    driver.setDrivetrain(drivetrain);
    driver.setAprilTagLocalization(aprilTagLocalization);
    driver.setCommandFactory(commandFactory);
    driver.setTurret(new Turret());
    driver.setIntake(new Intake());
    driver.setLedLights(ledLights);

    driver.setZones(zones);

    return driver;
  }

  @Override
  public Driver configureBindings() {
    // Use m_drivetrain here as needeed

    drivetrain.setDefaultCommand(
        drivetrain.gasPedalCommand(
            this::getRightTriggerAxis,
            this::getLeftTriggerAxis,
            this::getRightX,
            this::getLeftY,
            this::getLeftX,
            zones));

    ledLights.setDefaultCommand(ledLights.ledByShifts());

    this.a().onTrue(aprilTagLocalization.setTrust(true));
    this.a().onFalse(aprilTagLocalization.setTrust(false));

    this.povLeft().whileTrue(commandFactory.driveCircle());

    // Reset the field-centric heading on left bumper press.
    SmartDashboard.putNumber("Pose X", 0);
    SmartDashboard.putNumber("Pose Y", 0);

    this.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
    this.rightBumper()
        .onTrue(
            drivetrain.resetPose2d(
                new Pose2d(
                    SmartDashboard.getNumber("Pose X", 0),
                    SmartDashboard.getNumber("Pose Y", 0),
                    drivetrain.getRotation3d().toRotation2d())));

    return this;
  }
}
