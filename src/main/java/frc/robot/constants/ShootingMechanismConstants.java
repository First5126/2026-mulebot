package frc.robot.constants;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;

public class ShootingMechanismConstants {
  public static final Angle turretMaximumError = Degree.of(5);
  public static final LinearVelocity ballVelocity = MetersPerSecond.of(10);
  public static final LinearAcceleration gravity = MetersPerSecondPerSecond.of(9.8);
}
