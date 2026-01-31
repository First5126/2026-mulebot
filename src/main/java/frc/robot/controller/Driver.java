package frc.robot.controller;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.constants.ControllerConstants;
import frc.robot.subsystems.CommandFactory;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Turret;
import frc.robot.vision.AprilTagLocalization;

public class Driver extends CustomXboxController implements Controller{
    private final SwerveRequest.SwerveDriveBrake BRAKE = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt POINT = new SwerveRequest.PointWheelsAt();

    private CommandSwerveDrivetrain m_drivetrain;
    private AprilTagLocalization m_aprilTagLocalization;
    private CommandFactory m_commandFactory;
    private Intake m_intake;
    private final Turret m_turret;

    /**
     * Initializes the driver xbox controller.  All subsystems the controller will need to interact
     * with will need to be supplied to the constructor.
     */
    public Driver(
        final CommandSwerveDrivetrain swerveDrivetrain,
        final AprilTagLocalization aprilTagLocalization
    ) {
        super(ControllerConstants.DRIVER_CONTROLLER_PORT);

        this.m_drivetrain = swerveDrivetrain;
        this.m_aprilTagLocalization = aprilTagLocalization;
        this.m_commandFactory = new CommandFactory(m_drivetrain);
        this.m_intake = new Intake();
        this.m_turret = new Turret();

    }

    @Override
    public Driver configureBindings() {
                // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        m_drivetrain.setDefaultCommand(
            m_drivetrain.gasPedalCommand(
                this::getRightTriggerAxis,
                this::getLeftTriggerAxis,
                this::getRightX,
                this::getLeftY,
                this::getLeftX));
        
        this.a().onTrue(m_aprilTagLocalization.setTrust(true));
        this.a().onFalse(m_aprilTagLocalization.setTrust(false));

        this.povUp().onTrue(m_turret.goToPosition(Degrees.of(0)));
        this.povLeft().onTrue(m_turret.goToPosition(Degrees.of(90)));
        this.povRight().onTrue(m_turret.goToPosition(Degrees.of(180)));
        this.povLeft().whileTrue(m_commandFactory.driveCircle());

        // Reset the field-centric heading on left bumper press.
        this.leftBumper().onTrue(m_drivetrain.runOnce(m_drivetrain::seedFieldCentric));

        return this;
    }    
}
