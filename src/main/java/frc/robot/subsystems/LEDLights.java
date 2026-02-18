// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.core.CoreCANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FMS.StageData;
import frc.robot.FMS.Zones;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class LEDLights extends SubsystemBase {

  private static final int kCANdleCANbus = 0;
  private static final CANBus driveBaseCanivore = new CANBus("DriveBase");
  private static CoreCANdle m_candle = new CoreCANdle(kCANdleCANbus, driveBaseCanivore);
  private StageData m_stageData;

  private CANdleConfiguration m_configs = new CANdleConfiguration();

  private final RGBWColor CLEAR = new RGBWColor(255, 255, 255);
  private final RGBWColor RED = new RGBWColor(255, 0, 0);
  private final RGBWColor GREEN = new RGBWColor(0, 255, 0);
  private final RGBWColor BLUE = new RGBWColor(0, 0, 255);
  private final RGBWColor ORANGE = new RGBWColor(255, 157, 0);
  private final RGBWColor PURPLE = new RGBWColor(151, 0, 180);
  private final RGBWColor BLACK = new RGBWColor(0, 0, 0);
  private final int END_INDEX = 67;
  private final int START_INDEX = 0;
  private final RainbowAnimation rainbow = new RainbowAnimation(START_INDEX, END_INDEX);
  private SolidColor m_solidColorControl = new SolidColor(START_INDEX, END_INDEX);
  private boolean cleared = false;

  private final int COUNTDOWN_LED_START_INDEX = 0;
  private final int COUNTDOWN_LED_END_INDEX = 26;

  public LEDLights(StageData stageData) {
    m_stageData = stageData;
    m_candle.getConfigurator().apply(m_configs);
  }

  private void setRainbow() {
    m_candle.setControl(rainbow);
  }
  ;

  public void setRed() {
    m_candle.setControl(m_solidColorControl.withColor(RED));
  }

  public void setGreen() {
    m_candle.setControl(m_solidColorControl.withColor(GREEN));
  }

  public Command setBlue() {
    return run(() -> applyColor(BLUE));
  }

  public Command setPurple() {
    return run(() -> applyColor(PURPLE));
  }

  public Command stopRainbow() {
    return run(() -> m_candle.setControl(m_solidColorControl.withColor(RED)));
  }

  public void setClear() {}

  public Command ledOnZoneCommand(Zones zone) {
    return run(
        () -> {
          switch (zone.getZone()) {
            case ALLIANCE_ZONE:
              applyColor(BLUE);
              break;
            case NEUTRAL_ZONE:
              applyColor(PURPLE);
              break;
            case OPPONENT_ZONE:
              applyColor(RED);
              break;
            default:
              applyColor(GREEN);
              break;
          }
        });
  }

  public Command ledByShifts() {
    return run(
        () -> {
          Supplier<Integer> timeLeft = () -> m_stageData.getRemainingSecondsInStage();
          RGBWColor color = PURPLE;
          if (m_stageData.canScore()) {
            color = GREEN;
          } else {
            color = RED;
          }
          SmartDashboard.putBoolean("Time Left Condition Statment", timeLeft.get() < 5);
          SmartDashboard.putNumber("Time Left til Change", timeLeft.get());

          // must initialize the leds on when starting the stage
          if (m_stageData.getRemainingSecondsInStage()
              == StageData.getStage().getSecondsInStage()) {
            applyColorWithIndex(color, COUNTDOWN_LED_START_INDEX, COUNTDOWN_LED_END_INDEX);
          } else {
            // start removing colors from the led
            int ledCount = m_stageData.getRemainingSecondsInStage();
            SmartDashboard.putNumber("Ammount of leds", ledCount);
            int endIndex = (int) (COUNTDOWN_LED_END_INDEX - Math.ceil((double) ledCount / 2));
            int startIndex = (int) (COUNTDOWN_LED_START_INDEX + Math.floor((double) ledCount / 2));
            applyColorWithIndex(BLACK, COUNTDOWN_LED_START_INDEX, startIndex);
            applyColorWithIndex(BLACK, endIndex, END_INDEX);
          }
        });
  }

  public Command ledByMotion(
      DoubleSupplier rightTrigger,
      DoubleSupplier leftTrigger,
      DoubleSupplier rotationX,
      DoubleSupplier rotationY,
      DoubleSupplier joystickY,
      DoubleSupplier joystickX) {

    return run(
        () -> {
          boolean moving =
              rightTrigger.getAsDouble() > 0.05
                  || leftTrigger.getAsDouble() > 0.05
                  || rotationX.getAsDouble() > 0.05
                  || rotationY.getAsDouble() > 0.05
                  || joystickX.getAsDouble() > 0.05
                  || joystickY.getAsDouble() > 0.05;

          if (moving) {
            setGreen();
          } else {
            setRed();
          }
        });
  }

  private void applyColor(RGBWColor color) {
    m_candle.setControl(m_solidColorControl.withColor(color));
  }

  private void applyColorWithIndex(RGBWColor color, int start, int end) {
    m_candle.setControl(
        m_solidColorControl.withColor(color).withLEDStartIndex(start).withLEDEndIndex(end));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("RobotCentric", kCANdleCANbus);
  }
}
