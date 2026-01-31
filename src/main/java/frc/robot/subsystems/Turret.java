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
import frc.robot.constants.TurretConstants;

public class Turret extends SubsystemBase {
    private TalonFXS m_turretMotor = new TalonFXS(CANConstants.turretMotor);

    public Turret() {
        m_turretMotor.getConfigurator().apply(TurretConstants.getTalonFXSConfiguration());
    }

    public Command goToPosition(double position){

        return runOnce(() -> {
            m_turretMotor.setControl(new PositionVoltage(position));
        });

    }

}