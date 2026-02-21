package frc.robot.FMS;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.FMS.ShiftData.GameShift;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class ShiftDataGameShiftTest {
  @BeforeAll
  static void setupHal() {
    HAL.initialize(500, 0);
  }

  @AfterEach
  void resetDriverStation() {
    DriverStationSim.resetData();
    DriverStationSim.notifyNewData();
  }

  @Test
  void gameShiftDurationsMatchExpectedValues() {
    assertEquals(20, GameShift.Auto.getDuration());
    assertEquals(10, GameShift.TransitionShift.getDuration());
    assertEquals(25, GameShift.ShiftOne.getDuration());
    assertEquals(25, GameShift.ShiftTwo.getDuration());
    assertEquals(25, GameShift.ShiftThree.getDuration());
    assertEquals(25, GameShift.ShiftFour.getDuration());
    assertEquals(30, GameShift.Endgame.getDuration());
  }

  @Test
  void gameShiftStartAndEndTimesAreCumulativeAndContiguous() {
    assertEquals(20, GameShift.Auto.getStartTime());
    assertEquals(0, GameShift.Auto.getEndTime());

    assertEquals(140, GameShift.TransitionShift.getStartTime());
    assertEquals(130, GameShift.TransitionShift.getEndTime());

    assertEquals(130, GameShift.ShiftOne.getStartTime());
    assertEquals(105, GameShift.ShiftOne.getEndTime());

    assertEquals(105, GameShift.ShiftTwo.getStartTime());
    assertEquals(80, GameShift.ShiftTwo.getEndTime());

    assertEquals(80, GameShift.ShiftThree.getStartTime());
    assertEquals(55, GameShift.ShiftThree.getEndTime());

    assertEquals(55, GameShift.ShiftFour.getStartTime());
    assertEquals(30, GameShift.ShiftFour.getEndTime());

    assertEquals(30, GameShift.Endgame.getStartTime());
    assertEquals(0, GameShift.Endgame.getEndTime());
  }

  @Test
  void getShiftReturnsAutoWhenRobotIsInAutonomous() {
    setMatchState(true, 15.0);
    assertEquals(GameShift.Auto, ShiftData.getShift());
  }

  @Test
  void getShiftReturnsTransitionAtTeleopStart() {
    setMatchState(false, 140.0);
    assertEquals(GameShift.TransitionShift, ShiftData.getShift());
  }

  @Test
  void getShiftReturnsExpectedShiftAcrossTeleopTimeWindows() {
    setMatchState(false, 30.0);
    assertEquals(GameShift.Endgame, ShiftData.getShift());

    setMatchState(false, 40.0);
    assertEquals(GameShift.ShiftFour, ShiftData.getShift());

    setMatchState(false, 65.0);
    assertEquals(GameShift.ShiftThree, ShiftData.getShift());

    setMatchState(false, 100.0);
    assertEquals(GameShift.ShiftTwo, ShiftData.getShift());

    setMatchState(false, 130.0);
    assertEquals(GameShift.ShiftOne, ShiftData.getShift());
  }

  @Test
  void getTimeRemainingInShiftReturnsExpectedValueInAuto() {
    setMatchState(true, 15.0);
    assertEquals(15.0, ShiftData.getTimeRemainingInShift());
  }

  @Test
  void getTimeRemainingInShiftReturnsExpectedValueInTeleop() {
    // shift1 start at 130 and ends at 105
    setMatchState(false, 120.0);
    assertEquals(15.0, ShiftData.getTimeRemainingInShift());

    // transition starts at 140 and ends at 130
    setMatchState(false, 135.0);
    assertEquals(5.0, ShiftData.getTimeRemainingInShift());
  }

  @Test
  void getRemainingShiftPercentageReturnsExpectedValueInAuto() {
    setMatchState(true, 15.0);
    assertEquals(0.75, ShiftData.getRemainingShiftPercentage());
  }

  @Test
  void getRemainingShiftPercentageReturnsExpectedValueInTeleop() {
    // ShiftTwo duration 25, at t=100 we have 20 seconds left in shift.
    setMatchState(false, 100.0);
    assertEquals(0.8, ShiftData.getRemainingShiftPercentage());

    // Endgame duration 30, at t=30 we are at the start of endgame.
    setMatchState(false, 30.0);
    assertEquals(1.0, ShiftData.getRemainingShiftPercentage());

    // Endgame at t=0 has no time remaining.
    setMatchState(false, 0.0);
    assertEquals(0.0, ShiftData.getRemainingShiftPercentage());
  }

  @Test
  void getRemainingShiftPercentageIsClampedToUpperBound() {
    // Auto duration is 20; this would be 1.25 without clamping.
    setMatchState(true, 25.0);
    assertEquals(1.0, ShiftData.getRemainingShiftPercentage());
  }

  @Test
  void getRemainingShiftPercentageIsClampedToLowerBound() {
    // Negative match time should never produce a negative percentage.
    setMatchState(false, -5.0);
    assertEquals(0.0, ShiftData.getRemainingShiftPercentage());
  }

  private void setMatchState(boolean autonomous, double matchTimeSeconds) {
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.setAutonomous(autonomous);
    DriverStationSim.setMatchTime(matchTimeSeconds);
    DriverStationSim.notifyNewData();
  }
}
