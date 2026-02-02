package frc.robot.controller;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.constants.ControllerConstants;
import frc.robot.subsystems.CommandFactory;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Turret;
import frc.robot.vision.AprilTagLocalization;
import lombok.Getter;
import lombok.Setter;

public class Driver extends CustomXboxController implements Controller{
    // Singleton instance
    private static Driver INSTANCE;

    @Getter @Setter private CommandSwerveDrivetrain drivetrain;
    @Getter @Setter private AprilTagLocalization aprilTagLocalization;
    @Getter @Setter private CommandFactory commandFactory;
    @Getter @Setter private Intake intake = new Intake();
    @Getter @Setter private Turret turret = new Turret();

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
        CommandFactory commandFactory
    ){
        Driver driver = getInstance();
        driver.setDrivetrain(drivetrain);
        driver.setAprilTagLocalization(aprilTagLocalization);
        driver.setCommandFactory(commandFactory);
        driver.setTurret(new Turret());
        driver.setIntake(new Intake());

        return driver;
    }

    @Override
    public Driver configureBindings() {
        // Use m_drivetrain here as needed

        drivetrain.setDefaultCommand(
            drivetrain.gasPedalCommand(
                this::getRightTriggerAxis,
                this::getLeftTriggerAxis,
                this::getRightX,
                this::getLeftY,
                this::getLeftX));
        
        this.a().onTrue(aprilTagLocalization.setTrust(true));
        this.a().onFalse(aprilTagLocalization.setTrust(false));

        this.povUp().onTrue(turret.goToPosition(Degrees.of(0)));
        this.povLeft().onTrue(turret.goToPosition(Degrees.of(90)));
        this.povRight().onTrue(turret.goToPosition(Degrees.of(180)));
        this.povLeft().whileTrue(commandFactory.driveCircle());

        // Reset the field-centric heading on left bumper press.
        this.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        return this;
    }    
}
