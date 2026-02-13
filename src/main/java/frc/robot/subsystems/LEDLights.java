// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.nio.file.WatchEvent;
import java.util.function.Supplier;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.core.CoreCANdle;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.RobotCentric;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDLights extends SubsystemBase{

  private static final int kCANdleCANbus = 0;
  private static final CANBus driveBaseCanivore = new CANBus("DriveBase");
  private static CoreCANdle m_candle = new CoreCANdle(kCANdleCANbus, driveBaseCanivore);
  
  private CANdleConfiguration m_configs = new CANdleConfiguration();

  private final RGBWColor CLEAR = new RGBWColor(255, 255, 255);
  private final RGBWColor RED = new RGBWColor(255, 0, 0);
  private final RGBWColor GREEN = new RGBWColor(0, 255, 0);
  private final RGBWColor BLUE = new RGBWColor(0, 0, 255);
  private final RGBWColor ORANGE = new RGBWColor(255, 157, 0);
  private final RGBWColor PURPLE = new RGBWColor(151, 0, 180);
  private final RainbowAnimation rainbow = new RainbowAnimation(0, 67);
  private static SolidColor m_solidColorControl = new SolidColor(0, 67);
  private int counter = 0;  
      
        public LEDLights() {
          m_candle.getConfigurator().apply(m_configs);
        }
      
        private void setRainbow() {
              m_candle.setControl(rainbow); 
          };
      
        public void setSolidColor() {
           run(() -> m_candle.setControl(m_solidColorControl.withColor(GREEN)));
      }
    

      public void stopSolidColor() {
          run(() -> m_candle.setControl(m_solidColorControl.withColor(RED)));
  }


  public Command setBlue() {
        return run(() -> applyColor(BLUE));
  }

  public Command setRed() {
        return run(() -> applyColor(RED));
  }
  
  public Command setGreen() {
        return run(() -> applyColor(GREEN));
  }

  public Command driveLEDs(Supplier<Double> robotCentric, Supplier<Double> fieldCentric){
    return runOnce(() -> {
      counter ++;
      if(robotCentric.get() > 0.05 || fieldCentric.get() > 0.05){
        applyColor(GREEN);
      }
      else {
        applyColor(RED);
        counter = 0;
    }});
  }

  private void applyColor(RGBWColor color) {
          m_candle.setControl(m_solidColorControl.withColor(color));
  }

  @Override
  public void periodic() {
      SmartDashboard.putNumber("RobotCentric", kCANdleCANbus);
  }

}
