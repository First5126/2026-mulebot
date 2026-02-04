package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CANConstants;
import frc.robot.constants.TurretConstants;

public class Turret extends SubsystemBase {
    private final TalonFXS m_turretMotor = new TalonFXS(CANConstants.turretMotor,CANConstants.driveBaseCanivore);
    private final CANcoder m_turretEncoder = new CANcoder(CANConstants.turretEncoder,CANConstants.driveBaseCanivore);
    private final PositionVoltage m_positionControl = new PositionVoltage(0);

    public Turret() {
        TalonFXSConfiguration talonFXSConfiguration = new TalonFXSConfiguration();
        talonFXSConfiguration.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
        talonFXSConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        talonFXSConfiguration.ExternalFeedback.withFusedCANcoder(m_turretEncoder);
        talonFXSConfiguration.ExternalFeedback.RotorToSensorRatio = 4;
        talonFXSConfiguration.ExternalFeedback.SensorToMechanismRatio = 10;
        //talonFXSConfiguration.ExternalFeedback.FeedbackRemoteSensorID = CANConstants.turretEncoder;

        Slot0Configs slotConfigs = new Slot0Configs();
        slotConfigs.kP = 12;

        talonFXSConfiguration.Slot0 = slotConfigs;

        m_turretMotor.getConfigurator().apply(talonFXSConfiguration);
    }

    /**
     * Returns a command that rotates the turret to the specified position.
     * @param position The target angle (use WPILib Units, e.g. Units.Degrees.of(90))
     * @return a WPILib Command object to run once
     */
    public Command rotateToPosition(Angle position){
        // Convert all angles to degrees for clamping
        double minDegrees = TurretConstants.MIN_ANGLE.in(Degrees);
        double maxDegrees = TurretConstants.MAX_ANGLE.in(Degrees);
        double requestedDegrees = position.in(Degrees);

        double clampedDegrees = Math.max(minDegrees, Math.min(requestedDegrees, maxDegrees));
        // Construct the measure back in degrees
        Angle clampedPosition = Degrees.of(clampedDegrees);

        return runOnce(() -> {
            m_turretMotor.setControl(m_positionControl.withPosition(clampedPosition));
        });
    }

    @Override
    public void periodic() {
        double currentAngle = m_turretEncoder.getAbsolutePosition().getValueAsDouble() * 360.0;
        SmartDashboard.putNumber("Turret Angle (deg)", currentAngle);
    }
}