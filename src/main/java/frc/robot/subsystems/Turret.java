package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CANConstants;
import frc.robot.constants.TurretConstants;
import frc.robot.subsystems.ShootingMechanism.ShootingSolution;
import java.util.function.Supplier;

public class Turret extends SubsystemBase {
  private final TalonFXS m_turretMotor =
      new TalonFXS(CANConstants.turretMotor, CANConstants.driveBaseCanivore);
  // private final CANcoder m_turretEncoder =
  //  new CANcoder(CANConstants.turretEncoder, CANConstants.driveBaseCanivore);
  private final PositionVoltage m_positionControl = new PositionVoltage(0);
  private final VelocityVoltage m_velocityVoltage = new VelocityVoltage(0);
  private final DutyCycleOut m_dutyCycleOut = new DutyCycleOut(0);

  public Turret() {

    // CANcoderConfiguration canCoderConfiguration = new CANcoderConfiguration();
    // canCoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

    // m_turretEncoder.getConfigurator().apply(canCoderConfiguration);

    TalonFXSConfiguration talonFXSConfiguration = new TalonFXSConfiguration();
    talonFXSConfiguration.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
    talonFXSConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    talonFXSConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    // talonFXSConfiguration.ExternalFeedback.withFusedCANcoder(m_turretEncoder);
    talonFXSConfiguration.ExternalFeedback.RotorToSensorRatio = 4;
    talonFXSConfiguration.ExternalFeedback.SensorToMechanismRatio = 10;
    // This might work. Look into it before reanabling.
    // talonFXSConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
    // TurretConstants.MAX_ANGLE.in(Degrees);
    // talonFXSConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
    // TurretConstants.MIN_ANGLE.in(Degrees);
    // talonFXSConfiguration.ExternalFeedback.FeedbackRemoteSensorID = CANConstants.turretEncoder;

    Slot0Configs slotConfigs = new Slot0Configs();
    slotConfigs.kP = TurretConstants.kP;
    slotConfigs.kI = TurretConstants.kI;
    slotConfigs.kD = TurretConstants.kD;

    talonFXSConfiguration.Slot0 = slotConfigs;

    Slot1Configs slot1Configs = new Slot1Configs();
    slotConfigs.kP = 10;
    slotConfigs.kV = 3.84;

    talonFXSConfiguration.Slot1 = slot1Configs;

    m_turretMotor.getConfigurator().apply(talonFXSConfiguration);
  }

  public Command setSpeed(AngularVelocity velocity) {
    return runOnce(
      () -> {
        setVelocity(velocity);
      });
  }

  public Command stop() {
    return runOnce(
    () -> {
      brake();
    });
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
        });
  }

  public Command rotateToPosition(Supplier<ShootingSolution> shootingSolution) {
    return runOnce(
        () -> {
          setPosition(shootingSolution.get().predictedTurretAngle);
        });
  }

  public Command setZero() {
    return runOnce(
        () -> {
          m_turretMotor.setPosition(0);
        });
  }

  public Command moveToZero() {
    return runOnce(
        () -> {
          setPosition(Degrees.of(0));
        });
  }

  @Override
  public void periodic() {
    double currentAngle = m_turretMotor.getPosition().getValueAsDouble() * 360.0;
    SmartDashboard.putNumber("Turret Angle (deg)", currentAngle);
  }

  public double getTimeFromDistance(Supplier<Double> distance) {
    return TurretConstants.DISTANCE_TO_TIME_INTERPOLATOR.get(distance.get());
  }

  public double findTimeFromFuelShootingDistance(double distance) {
    throw new UnsupportedOperationException(
        "TODO: Implement findTimeFromFuelShootingDistance(double distance) based on shooter model\"");
  }

  public Angle getPosition() {
    return m_turretMotor.getPosition().getValue();
  }

  private void setPosition(final Angle position) {
    // Convert all angles to degrees for clamping
    double minDegrees = TurretConstants.MIN_ANGLE.in(Degrees);
    double maxDegrees = TurretConstants.MAX_ANGLE.in(Degrees);
    double requestedDegrees = position.in(Degrees);

    double clampedDegrees = Math.max(minDegrees, Math.min(requestedDegrees, maxDegrees));
    // Construct the measure back in degrees
    Angle clampedPosition = Degrees.of(clampedDegrees);

    m_turretMotor.setControl(m_positionControl.withPosition(clampedPosition).withSlot(0));
  }

  private void setVelocity(AngularVelocity velocity) {
    m_turretMotor.setControl(m_velocityVoltage.withVelocity(velocity).withSlot(1));
  }

  private void brake() {
    m_turretMotor.setControl(m_dutyCycleOut.withOutput(0));
  }
}
