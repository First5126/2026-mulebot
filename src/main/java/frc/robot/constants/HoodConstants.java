package frc.robot.constants;

import java.util.Map;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class HoodConstants {
  public static double kP = 0.0;
  public static double kI = 0.0;
  public static double kD = 0.0;
  public static double kV = 0.0;

  // TODO: Very important to fix this so we dont rip the hood off
  public static final InterpolatingDoubleTreeMap DISTANCE_TO_ANGLE_INTERPOLATOR =
      new InterpolatingDoubleTreeMap()
          .ofEntries(Map.entry(0.0, 0.0), Map.entry(2.0, 0.5), Map.entry(5.0, 1.0));
}
