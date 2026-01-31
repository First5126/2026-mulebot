package frc.robot.constants;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class TurretConstants {
        

    public static Slot0Configs getSlotConfigs() {
        Slot0Configs slotConfigs = new Slot0Configs();
        slotConfigs.kP = 12;

        return slotConfigs;
    }


    public static TalonFXSConfiguration getTalonFXSConfiguration() {
        TalonFXSConfiguration talonFXSConfiguration = new TalonFXSConfiguration();
        talonFXSConfiguration.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
        talonFXSConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        talonFXSConfiguration.ExternalFeedback.RotorToSensorRatio = 4;
        talonFXSConfiguration.ExternalFeedback.SensorToMechanismRatio = 10;
        talonFXSConfiguration.ExternalFeedback.FeedbackRemoteSensorID = CANConstants.turretEncoder;

        talonFXSConfiguration.Slot0 = TurretConstants.getSlotConfigs();

        return talonFXSConfiguration;
    }

}