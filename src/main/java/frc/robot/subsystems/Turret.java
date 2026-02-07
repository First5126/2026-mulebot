package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CANConstants;
import frc.robot.constants.TurretConstants;
import java.util.function.Supplier;

public class Turret extends SubsystemBase {
  private final TalonFXS m_turretMotor =
      new TalonFXS(CANConstants.turretMotor, CANConstants.driveBaseCanivore);
  private final CANcoder m_turretEncoder =
      new CANcoder(CANConstants.turretEncoder, CANConstants.driveBaseCanivore);
  private final PositionVoltage m_positionControl = new PositionVoltage(0);

  public Turret() {

    CANcoderConfiguration canCoderConfiguration = new CANcoderConfiguration();
    canCoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

    m_turretEncoder.getConfigurator().apply(canCoderConfiguration);

    TalonFXSConfiguration talonFXSConfiguration = new TalonFXSConfiguration();
    talonFXSConfiguration.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
    talonFXSConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    talonFXSConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    talonFXSConfiguration.ExternalFeedback.withFusedCANcoder(m_turretEncoder);
    talonFXSConfiguration.ExternalFeedback.RotorToSensorRatio = 4;
    talonFXSConfiguration.ExternalFeedback.SensorToMechanismRatio = 10;
    // This might work. Look into it before reanabling.
    // talonFXSConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
    // TurretConstants.MAX_ANGLE.in(Degrees);
    // talonFXSConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
    // TurretConstants.MIN_ANGLE.in(Degrees);
    // talonFXSConfiguration.ExternalFeedback.FeedbackRemoteSensorID = CANConstants.turretEncoder;

    Slot0Configs slotConfigs = new Slot0Configs();
    slotConfigs.kP = 48;

    talonFXSConfiguration.Slot0 = slotConfigs;

    m_turretMotor.getConfigurator().apply(talonFXSConfiguration);
  }

  /**
   * Returns a command that rotates the turret to the specified position.
   *
   * @param position The target angle (use WPILib Units, e.g. Units.Degrees.of(90))
   * @return a WPILib Command object to run once
   */
  public Command rotateToPosition(Angle position) {
    return runOnce(
        () -> {
          setPosition(position);
          ;
        });
  }

  @Override
  public void periodic() {
    double currentAngle = m_turretMotor.getPosition().getValueAsDouble() * 360.0;
    SmartDashboard.putNumber("Turret Angle (deg)", currentAngle);
  }

  public Double getTimeFromDistance(Supplier<Double> distance) {
    return TurretConstants.DISTANCE_TO_TIME_INTERPOLATOR.get(distance.get());
  }

  public Command trackTargetPoseCommand(Supplier<Pose2d> robotPose, Supplier<Pose2d> targetPose) {
    return run(
        () -> {
          SmartDashboard.putBoolean("robotPose Valid", robotPose.get() != null);
          SmartDashboard.putBoolean("targetPose Valid", targetPose.get() != null);
          if (robotPose.get() != null && targetPose.get() != null) {
            Pose2d turretPose = robotPose.get().plus(TurretConstants.TURRET_OFFSET);

            double distanceX = targetPose.get().getX() - turretPose.getX();
            double distanceY = targetPose.get().getY() - turretPose.getY();

            Rotation2d fieldRelativeAngle =
                Rotation2d.fromRadians(Math.atan2(distanceY, distanceX));

            Rotation2d robotRelativeAngle = fieldRelativeAngle.minus(robotPose.get().getRotation());

            SmartDashboard.putNumber("Turret distanceX", distanceX);
            SmartDashboard.putNumber("Turret distanceY", distanceY);
            SmartDashboard.putNumber(
                "Turret fieldRelativeAngle in Degrees", fieldRelativeAngle.getDegrees());
            SmartDashboard.putNumber(
                "Turret robotRelativeAngle in Degrees", robotRelativeAngle.getDegrees());
            SmartDashboard.putString(
                "Turret Target Poseition",
                "X: " + targetPose.get().getX() + " Y: " + targetPose.get().getY());
            SmartDashboard.putString(
                "Turret Current Position",
                "X: " + robotPose.get().getX() + " Y: " + robotPose.get().getY());

            setPosition(robotRelativeAngle.getMeasure());
          }
        });
  }

  public void trackTargetPose(Supplier<Pose2d> robotPose, Supplier<Pose2d> targetPose) {
    SmartDashboard.putBoolean("robotPose Valid", robotPose.get() != null);
    SmartDashboard.putBoolean("targetPose Valid", targetPose.get() != null);
    if (robotPose.get() != null && targetPose.get() != null) {
      Pose2d turretPose = robotPose.get().plus(TurretConstants.TURRET_OFFSET);

      double distanceX = targetPose.get().getX() - turretPose.getX();
      double distanceY = targetPose.get().getY() - turretPose.getY();

      Rotation2d fieldRelativeAngle = Rotation2d.fromRadians(Math.atan2(distanceY, distanceX));

      Rotation2d robotRelativeAngle = fieldRelativeAngle.minus(robotPose.get().getRotation());

      SmartDashboard.putNumber("Turret distanceX", distanceX);
      SmartDashboard.putNumber("Turret distanceY", distanceY);
      SmartDashboard.putNumber(
          "Turret fieldRelativeAngle in Degrees", fieldRelativeAngle.getDegrees());
      SmartDashboard.putNumber(
          "Turret robotRelativeAngle in Degrees", robotRelativeAngle.getDegrees());
      SmartDashboard.putString(
          "Turret Target Poseition",
          "X: " + targetPose.get().getX() + " Y: " + targetPose.get().getY());
      SmartDashboard.putString(
          "Turret Current Position",
          "X: " + robotPose.get().getX() + " Y: " + robotPose.get().getY());

      setPosition(robotRelativeAngle.getMeasure());
    }
  }

  public double getDistanceFromHub(Supplier<Pose2d> robotPose, Zones zone) {
    Translation2d hubTranslation2d = zone.getTurretShootingPose().getTranslation();
    Translation2d botTranslation2d = robotPose.get().getTranslation();
    return hubTranslation2d.getDistance(botTranslation2d);
  }

  public double findTimeFromFuelShootingDistance(double distance) {
    return 0.0;
  }

  // takes 1 pose and then finds out bassed on your current pose and you going in a cirtain
  // dirrecton and bassed on the time it takes it will give you the calculated Pose2d
  public Pose2d calculatePredictedPose2d(
      CommandSwerveDrivetrain drivetrain, Supplier<Double> time) {
    Pose2d pose = drivetrain.getPose2d();
    double currentX = pose.getX();
    double currentY = pose.getY();

    double distance =
        time.get()
            * Math.sqrt(
                Math.pow(drivetrain.getSpeeds().vxMetersPerSecond, 2)
                    + Math.pow(drivetrain.getSpeeds().vyMetersPerSecond, 2));
    Rotation2d rotation =
        new Rotation2d(
            Math.atan2(
                drivetrain.getSpeeds().vyMetersPerSecond,
                drivetrain.getSpeeds().vxMetersPerSecond));
    rotation.plus(drivetrain.getPose2d().getRotation());

    SmartDashboard.putNumber("Rotation Calculation Turret", rotation.getDegrees());

    double predictedX = currentX + distance * Math.cos(rotation.getRadians());
    double predictedY = currentY + distance * Math.sin(rotation.getRadians());

    Pose2d predictedPose2d =
        new Pose2d(
            predictedX,
            predictedY,
            new Rotation2d(Radians.of(drivetrain.getPose2d().getRotation().getRadians())));

    return predictedPose2d;
  }

  private void setPosition(final Angle position) {
    // Convert all angles to degrees for clamping
    double minDegrees = TurretConstants.MIN_ANGLE.in(Degrees);
    double maxDegrees = TurretConstants.MAX_ANGLE.in(Degrees);
    double requestedDegrees = position.in(Degrees);

    double clampedDegrees = Math.max(minDegrees, Math.min(requestedDegrees, maxDegrees));
    // Construct the measure back in degrees
    Angle clampedPosition = Degrees.of(clampedDegrees);

    m_turretMotor.setControl(m_positionControl.withPosition(clampedPosition));
  }
}
