// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FMS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;

/** Class for managing the stages the robot can be within during the match */
public class StageData {
  /*
   * Enumeration to keep track of the shifts including the time in each shift.  Do note the computation
   * of the seconds utilized in this enum are based on a running clock time and not a remaining clock time.
   * It simplifies the usage of the data.
   */
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

    /**
     * Get the seconds that are in the current stage
     *
     * @return {@int} an integer value representing the seconds that exist for the stage
     */
    public int getSecondsInStage() {
      return m_secondsInStage;
    }

    /**
     * Get the seconds at which the stage will be over
     *
     * @return {@int} an integer indicating at what seconds into the match will the stage be over
     */
    public int getStageOverAtSeconds() {
      return m_stageOverAtSeconds;
    }
  }

  /**
   * Get the current stage in the match.
   *
   * @return {@GameStage} stage related to the current gameplay
   */
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

  /**
   * Returns if the robot can currently score on the hub. This takes into account who is the first
   * active alliance and determing if you can store on 1/3 shifts or 2/4 shifts.
   *
   * @return {@boolean} returns {true} if the robot can score on the hub; otherwise, {false} is
   *     returned
   */
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

  // helper for getting the current alliance
  private DriverStation.Alliance getCurentAlliance() {
    return DriverStation.getAlliance().orElse(null);
  }

  // return who is the first active alliance based on the game data.  if no game data is present
  // the first aliance is determined by the current aliance selected in driverstation.  until
  // logic is further vetted, the default may reverse but until utilized, we'll wait to see.
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
}
