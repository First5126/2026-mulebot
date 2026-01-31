package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CANConstants;

public class Turret extends SubsystemBase {
    private TalonFXS m_turretMotor = new TalonFXS(CANConstants.turretMotor,CANConstants.driveBaseCanvioreName);
    private CANcoder m_turretEnncoder = new CANcoder(CANConstants.turretEncoder,CANConstants.driveBaseCanvioreName);
    private PositionVoltage m_positionControl = new PositionVoltage(0);

    public Turret() {
        // Get base TalonFXS configuration from TurretConstants to avoid duplication
        TalonFXSConfiguration talonFXSConfiguration = TurretConstants.getTalonFXSConfiguration();

        // Ensure this instance's CANcoder is used as the fused feedback sensor
        talonFXSConfiguration.ExternalFeedback.withFusedCANcoder(m_turretEnncoder);

        // Get PID slot configuration from TurretConstants
        Slot0Configs slotConfigs = TurretConstants.getSlotConfigs();
        talonFXSConfiguration.Slot0 = slotConfigs;

        m_turretMotor.getConfigurator().apply(talonFXSConfiguration);
    }

    public Command goToPosition(Angle position){

        return runOnce(() -> {
            m_turretMotor.setControl(m_positionControl.withPosition(position));
        });

    }

}