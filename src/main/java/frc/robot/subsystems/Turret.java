package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CANConstants;

public class Turret extends SubsystemBase {
    private TalonFXS m_turretMotor = new TalonFXS(CANConstants.turretMotor);

    public Turret() {
        TalonFXSConfiguration talonFXSConfiguration = new TalonFXSConfiguration();
        talonFXSConfiguration.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
        talonFXSConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        talonFXSConfiguration.ExternalFeedback.RotorToSensorRatio = 4;
        talonFXSConfiguration.ExternalFeedback.SensorToMechanismRatio = 10;
        talonFXSConfiguration.ExternalFeedback.FeedbackRemoteSensorID = CANConstants.turretEncoder;

        Slot0Configs slotConfigs = new Slot0Configs();
        slotConfigs.kP = 12;

        talonFXSConfiguration.Slot0 = slotConfigs;

        m_turretMotor.getConfigurator().apply(talonFXSConfiguration);
    }

    public Command goToPosition(double position){

        return runOnce(() -> {
            m_turretMotor.setControl(new PositionVoltage(position));
        });

    }

}