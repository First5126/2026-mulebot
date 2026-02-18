// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FMS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;

public class StageData {

  public static GameStage getStage() {
    boolean auto = DriverStation.isAutonomous();
    if (auto) return GameStage.Auto;

    double gameTime = DriverStation.getMatchTime();
    if (gameTime < 30) return GameStage.Endgame;
    else if (gameTime < 55) return GameStage.ShiftFour;
    else if (gameTime < 80) return GameStage.ShiftThree;
    else if (gameTime < 105) return GameStage.ShiftTwo;
    else if (gameTime < 130) return GameStage.ShiftOne;
    else return GameStage.TransitionShift;
  }

  private static Alliance getFirstActiveAlliance() {
    String message = DriverStation.getGameSpecificMessage();
    if (message.length() > 0) {
      // capture the first inactive alliance
      char cVal = message.charAt(0);
      if (cVal == 'R') {
        return Alliance.Blue;
      } else {
        return Alliance.Red;
      }
    }
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
        ? Alliance.Red
        : Alliance.Blue;
  }

  /**
   * @return int remaining seconds for the current stage
   */
  public int getRemainingSecondsInStage() {
    if (Timer.getMatchTime() == -1.0) {
      return 0;
    } else {
      return StageData.getStage().getStageOverAtSeconds() - (150 - (int) Timer.getMatchTime());
    }
  }

  private DriverStation.Alliance getCurentAlliance() {
    return DriverStation.getAlliance().orElse(null);
  }

  public enum GameStage {
    Auto(20),
    TransitionShift(10),
    ShiftOne(25),
    ShiftTwo(25),
    ShiftThree(25),
    ShiftFour(25),
    Endgame(30);

    private final int m_secondsInStage;
    private final int m_stageOverAtSeconds;

    GameStage(int secondsInStage) {
      this.m_secondsInStage = secondsInStage;
      this.m_stageOverAtSeconds = computeStageOverAtSeconds(this);
    }

    private static int computeStageOverAtSeconds(GameStage current) {
      int total = 0;
      for (GameStage s : GameStage.values()) {
        total += s.m_secondsInStage;
        if (s == current) {
          return total;
        }
      }
      throw new IllegalStateException("Stage not found: " + current);
    }

    public int getSecondsInStage() {
      return m_secondsInStage;
    }

    public int getStageOverAtSeconds() {
      return m_stageOverAtSeconds;
    }
  }

  public boolean canScore() {
    GameStage stage = getStage();

    if (stage == GameStage.Auto || stage == GameStage.Endgame || stage == GameStage.TransitionShift)
      return true;

    // if my current alliance is the first to be active, I can shoot in shift 1 and 3
    if (getCurentAlliance() == StageData.getFirstActiveAlliance()) {
      switch (stage) {
        case ShiftOne:
        case ShiftThree:
          return true;
        default:
          return false;
      }

    } else {
      // I can shoot in shift 2 and 4
      switch (stage) {
        case ShiftTwo:
        case ShiftFour:
          return true;
        default:
          return false;
      }
    }
  }
}
