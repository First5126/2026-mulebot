package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.LinearVelocity;

public class FlyWheel {
  // This class currently dosent have the motor contorl or other methods but simpily stores the
  // speed
  // for use in the shooting mechanism

  private LinearVelocity speed = MetersPerSecond.of(10);

  public void setSpeed(LinearVelocity newSpeed) {
    speed = newSpeed;
  }

  public LinearVelocity getSpeed() {
    return speed;
  }
}
