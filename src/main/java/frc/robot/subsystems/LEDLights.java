// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.core.CoreCANdle;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDLights extends SubsystemBase{

  private static final int kCANdleCANbus = 0;
  final private CoreCANdle m_candle = new CoreCANdle(kCANdleCANbus);
  
  private CANdleConfiguration m_configs = new CANdleConfiguration();

  private final RGBWColor WHITE = new RGBWColor(255, 255, 255);
  private final RGBWColor CLEAR = new RGBWColor(0, 0, 0);
  private final RGBWColor RED = new RGBWColor(255, 0, 0);
  private final RGBWColor GREEN = new RGBWColor(0, 255, 0);
  private final RGBWColor BLUE = new RGBWColor(0, 0, 255);
  private final RGBWColor ORANGE = new RGBWColor(255, 157, 0);
  private final RGBWColor PURPLE = new RGBWColor(151, 0, 180);

  private final XboxController m_controller;

  public LEDLights(XboxController controller) {
    m_controller = controller;
    m_candle.getConfigurator().apply(m_configs);
  }

  @Override
    public void periodic() {
        if(m_controller.getLeftX() > 1 || m_controller.getRightX() > 1 || m_controller.getLeftY() > 1 || m_controller.getRightY() > 1){
          final RainbowAnimation rainbow = new RainbowAnimation(8, 67);
          m_candle.setControl(rainbow);
        }
        else{
          applyColor(WHITE);
        }
    }

  public void applyColor(RGBWColor color) {
          SolidColor sc = new SolidColor(0, 25).withColor(color);
          m_candle.setControl(sc);
  }


}
