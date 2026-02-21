// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FMS;

import edu.wpi.first.wpilibj.DriverStation;
import java.util.Optional;

public class ShiftData {
  private static DriverStation.Alliance m_firstActiveAlliance;
  private static DriverStation.Alliance m_ourAlliance;

  /**
   * Assigns the values for our alliance and the first active alliance. Should be called every time
   * getFirstActiveAlliance is called, at least until the data is found.
   */
  private static void getFMSData() {
    if (m_ourAlliance == null) {
      // Gets the alliance from driver's station, and assigns it to m_ourAlliance.
      Optional<DriverStation.Alliance> ourAlliance = DriverStation.getAlliance();
      if (ourAlliance.isPresent()) {
        m_ourAlliance = ourAlliance.get();
      }
    }

    if (m_firstActiveAlliance == null) {
      // Gets the alliance which has the second shift.
      String gameData = DriverStation.getGameSpecificMessage();
      if (gameData.length() < 1) return;

      switch (gameData.charAt(0)) {
        // If 'R', Blue has the first shift.
        case 'R':
          m_firstActiveAlliance = DriverStation.Alliance.Blue;
          break;
        // If 'B', Red has the first shift.
        case 'B':
          m_firstActiveAlliance = DriverStation.Alliance.Red;
          break;
        // If it somehow gets here, then somehow we recived invalid data, so don't modify the value.
        default:
          m_firstActiveAlliance = null;
          break;
      }
    }
  }

  /**
   * Enumeration to keep track of the shifts including the time in each shift. Shift duration is the
   * shift's length. Start time is the time remaining in the match when the shift starts. All times
   * are pulled from the <a
   * href="https://firstfrc.blob.core.windows.net/frc2026/Manual/2026GameManual.pdf">FRC Game
   * Manual</a>.
   */
  public enum GameShift {
    Auto(20, 20),
    TransitionShift(10, 140),
    ShiftOne(25, 130),
    ShiftTwo(25, 105),
    ShiftThree(25, 80),
    ShiftFour(25, 55),
    Endgame(30, 30);

    private final int shiftDuration;
    private final int startTime;

    GameShift(int shiftDuration, int startTime) {
      this.shiftDuration = shiftDuration;
      this.startTime = startTime;
    }

    /**
     * Get the duration of the stage.
     *
     * @return int an integer value representing the duration of the stage.
     */
    public int getShiftDuration() {
      return shiftDuration;
    }

    /**
     * Get the start time for the stage.
     *
     * @return int an integer value representing the start time of the stage.
     */
    public int getShiftStartTime() {
      return startTime;
    }
  }

  /**
   * Get the current shift in the match.
   *
   * @return GameShift shift related to the current gameplay
   */
  public static GameShift getShift() {
    // If we are in autonomous, we don't have to check what the remaining time is.
    boolean auto = DriverStation.isAutonomous();
    if (auto) return GameShift.Auto;

    // We aren't in auto, so compare the current match time to each shift's start time.
    double currentTime = DriverStation.getMatchTime();
    if (currentTime < GameShift.Endgame.getShiftStartTime()) return GameShift.Endgame;

    if (currentTime < GameShift.ShiftFour.getShiftStartTime()) return GameShift.ShiftFour;

    if (currentTime < GameShift.ShiftThree.getShiftStartTime()) return GameShift.ShiftThree;

    if (currentTime < GameShift.ShiftTwo.getShiftStartTime()) return GameShift.ShiftTwo;

    if (currentTime < GameShift.ShiftOne.getShiftStartTime()) return GameShift.ShiftOne;

    // If we made it here, it either has to be the transition shift, or we're running the bot in
    // normal teleop.
    return GameShift.TransitionShift;
  }

  /**
   * Gets if our alliance is the first active alliance.
   *
   * @return boolean representing if our alliance has the first enabled hub.
   */
  public static boolean getFirstActiveAlliance() {
    getFMSData();

    // Make sure the FMS data is availible before comparing values.
    if (m_ourAlliance == null || m_firstActiveAlliance == null) return false;

    // Since we are already getting the first active alliance, we can compare it to ours
    // to check if we're the first active
    return m_ourAlliance == m_firstActiveAlliance;
  }

  public static boolean canScore() {
    GameShift stage = getShift();

    // Check for if both hubs are active
    if (stage == GameShift.Auto
        || stage == GameShift.Endgame
        || stage == GameShift.TransitionShift) {
      return true;
    }

    boolean firstActive = getFirstActiveAlliance();
    switch (stage) {
      case ShiftOne:
        return firstActive;

      case ShiftTwo:
        return !firstActive;

      case ShiftThree:
        return firstActive;

      case ShiftFour:
        return !firstActive;

      default:
        return false;
    }
  }

  /**
   * Calculates the time remaining in the current shift.
   *
   * @return double representing the seconds remaining in the shift.
   */
  public static double getTimeRemainingInShift() {
    GameShift currentShift = getShift();
    double currentTime = DriverStation.getMatchTime();

    // Get the time elapsed in the current shift, and subtract it from the shift duration
    // to get the time remaining in the shift.
    return Math.max(
        currentShift.shiftDuration - (currentShift.getShiftStartTime() - currentTime), 0);
  }
}
