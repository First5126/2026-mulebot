// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.hardware.core.CoreCANdle;

import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDLights extends SubsystemBase{

  private static final int kCANdleCANbus = 0;
  final private CoreCANdle candle = new CoreCANdle(kCANdleCANbus);

  
  
 private CANdleConfiguration configs = new CANdleConfiguration();

  private static final Color8Bit WHITE = new Color8Bit(255, 255, 255);
  private static final Color8Bit CLEAR = new Color8Bit(0, 0, 0);
  private static final Color8Bit RED = new Color8Bit(255, 0, 0);
  private static final Color8Bit GREEN = new Color8Bit(0, 255, 0);
  private static final Color8Bit BLUE = new Color8Bit(0, 0, 255);
  private static final Color8Bit ORANGE = new Color8Bit(255, 157, 0);
  private static final Color8Bit PURPLE = new Color8Bit(151, 0, 180);



  public LEDLights() {
    candle.getConfigurator().apply(configs);
  }


}




  /**
   * Apply color to full set of LEDs on the robot
   *
   * @return a Command for applying a color
   */

