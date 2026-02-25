package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.FMS.ShiftData;
import frc.robot.FMS.Zones;
import frc.robot.constants.GoalPoseConstants.GoalPose;
import frc.robot.constants.HoodConstants;
import frc.robot.constants.ShootingMechanismConstants;
import frc.robot.constants.TurretConstants;

import java.util.Optional;
import java.util.function.Supplier;

public class ShootingMechanism extends SubsystemBase {
  public static class ShootingSolution {
    public Angle predictedHoodAngle;
    public Angle predictedTurretAngle;
    public LinearVelocity flyWheelSpeed;

    public ShootingSolution(Angle hoodAngle, Angle turretAngle, LinearVelocity flyWheelSpeed) {
      predictedHoodAngle = hoodAngle;
      predictedTurretAngle = turretAngle;
      this.flyWheelSpeed = flyWheelSpeed;
    }
  }

  public final Trigger canShoot;
  private Turret m_turret;
  private Hood m_hood;
  private ShootingSolution m_currentShootingSolution =
      new ShootingSolution(Degrees.of(0), Degrees.of(0), MetersPerSecond.of(0));
  private CommandSwerveDrivetrain m_drivetrain;
  private Zones m_zone;
  private FlyWheel m_flyWheel;
  
    public ShootingMechanism(Turret m_turret, CommandSwerveDrivetrain m_drivetrain, Zones m_zone, FlyWheel m_flyWheel) {
      this.m_turret = m_turret;
      this.m_drivetrain = m_drivetrain;
      this.m_zone = m_zone;
      this.m_flyWheel = m_flyWheel;
    // TODO: get the hood too

    canShoot = new Trigger(this::canShootFuel);

    //SmartDashboard.putNumber("Ball Velocity", 7.5);
  }

  public Trigger getCanShoot() {
    return canShoot;
  }

  public ShootingSolution getShootingSolution() {
    return m_currentShootingSolution;
  }

  private Optional<Angle> getHoodAngle(Distance distance, LinearVelocity ballSpeed, Distance verticalOffset) {

    double arcSin = Math.asin((distance.in(Meters) * ShootingMechanismConstants.gravity.in(MetersPerSecondPerSecond))/
      (ballSpeed.in(MetersPerSecond)*ballSpeed.in(MetersPerSecond))) / 2;

    SmartDashboard.putBoolean("Shot Possible", !Double.isNaN(arcSin));
    if (Double.isNaN(arcSin)) return Optional.empty(); // Return simpily a lower limit
    else {
      Angle calculatedAngle = Degrees.of(90).minus(Radians.of(arcSin)); // get the final calculation and the complemntary

      return Optional.ofNullable(withinHoodBounds(calculatedAngle)?calculatedAngle:null);     
    }
  }

  private Time getAirTime(Angle angle, LinearVelocity speed) {
    return Seconds.of(
      2 * (speed.in(MetersPerSecond)*Math.sin(angle.in(Radians))) / ShootingMechanismConstants.gravity.in(MetersPerSecondPerSecond)
    );
  }

  private boolean withinHoodBounds(Angle angle) {
    return ShootingMechanismConstants.minimumHoodAngle.in(Degrees) <= angle.in(Degrees) &&
    angle.in(Degrees) <= ShootingMechanismConstants.maximumHoodAngle.in(Degrees);
  }

  private Distance getDistance(Angle angle, LinearVelocity speed) {
    return Meters.of(speed.in(MetersPerSecond)*Math.cos(angle.in(Radians))*2*((speed.in(MetersPerSecond)*Math.sin(angle.in(Radians)))/ShootingMechanismConstants.gravity.in(MetersPerSecondPerSecond)));
  }

  private Angle getClosestHoodAngle(LinearVelocity speed, Distance goalDistance) {
      Distance lowerDistance = getDistance(ShootingMechanismConstants.minimumHoodAngle, speed);
      Distance upperDistance = getDistance(ShootingMechanismConstants.maximumDistanceHoodAngle, speed);

      double lowerDifference = Math.abs(goalDistance.minus(lowerDistance).in(Meters));
      double upperDifference = Math.abs(goalDistance.minus(upperDistance).in(Meters));

      return lowerDifference<upperDifference?ShootingMechanismConstants.minimumHoodAngle:ShootingMechanismConstants.maximumDistanceHoodAngle;
  }

  private LinearVelocity getSpeed(Distance distance, Angle angle) {
      return MetersPerSecond.of(Math.sqrt((distance.in(Meters)*ShootingMechanismConstants.gravity.in(MetersPerSecondPerSecond))/(2*Math.cos(angle.in(Radians)*Math.sin(angle.in(Radians))))));
  }
  /**
   * @param robotPoseSupplier The current pose of the robot
   * @param speed The chassis speeds of the drivetrain
   * @param goalPoseSupplier The goal pose of the target
   * @return A shooting soltuion {@link frc.robot.subsystems.ShootingMechanism.ShootingSolution}
   *     that contains the predicted angle for the hood and turret
   */
  private ShootingSolution getShootingSolution(
      Supplier<Pose2d> robotPoseSupplier,
      Supplier<ChassisSpeeds> speed,
      Supplier<GoalPose> goalPoseSupplier) {

    // check to see if our suppliers are valid
    if (robotPoseSupplier.get() != null
        && goalPoseSupplier.get() != null
        && speed.get() != null) {
      SmartDashboard.putBoolean("Valid Shooting Solution", true);

      // retreive the value of all the suppliers
      Pose2d robotPose = robotPoseSupplier.get();
      ChassisSpeeds robotSpeeds = speed.get();
      GoalPose goalPose = goalPoseSupplier.get();
      
      Pose2d targetPose = goalPose.pose;

      // find air time from distance
      Distance distanceToTarget = Meters.of(robotPose.getTranslation().getDistance(targetPose.getTranslation()));

      // inital hood angle
      Optional<Angle> optionalHoodAngle = getHoodAngle(distanceToTarget,m_flyWheel.getSpeed(),goalPose.verticalOffset);
      Angle hoodAngle;
      LinearVelocity ballSpeed = m_flyWheel.getSpeed();

      SmartDashboard.putBoolean("Optional Hood Angle Is Present", optionalHoodAngle.isPresent());
      if (optionalHoodAngle.isEmpty()) {
        // if we get a new hood angle we need to calculate a new ball speed
        hoodAngle = getClosestHoodAngle(m_flyWheel.getSpeed(), distanceToTarget);
        ballSpeed = getSpeed(distanceToTarget,hoodAngle);
      } 
      else hoodAngle = optionalHoodAngle.get();

      Time airTime = getAirTime(hoodAngle, m_flyWheel.getSpeed());
      SmartDashboard.putNumber("Air Time (S)", airTime.in(Seconds));

      // find how far we travel by the time the ball will reach the target
      double predicatedDistance =
          airTime.in(Seconds) * Math.hypot(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond);

      // find the angle of the the speeds that are currently in robotcentric
      Rotation2d rotation =
          new Rotation2d(Math.atan2(robotSpeeds.vyMetersPerSecond, robotSpeeds.vxMetersPerSecond));

      // add the angle of the speeds to get the field centric velocity angle
      rotation = rotation.plus(robotPose.getRotation());

      // find the predicated x and y of our robot pose
      double predictedX = robotPose.getX() + predicatedDistance * Math.cos(rotation.getRadians());
      double predictedY = robotPose.getY() + predicatedDistance * Math.sin(rotation.getRadians());

      // get the turret pose
      Pose2d turretPose =
          new Pose2d(predictedX, predictedY, new Rotation2d()).plus(TurretConstants.TURRET_OFFSET);

      // find the distance to target from predicted pose
      double targetDistanceX = targetPose.getX() - turretPose.getX();
      double targetDistanceY = targetPose.getY() - turretPose.getY();

      Distance totalDistance = Meters.of(Math.hypot(targetDistanceX, targetDistanceY));

      // find the field relative angle from the distance
      Rotation2d fieldRelativeAngle =
          Rotation2d.fromRadians(Math.atan2(targetDistanceY, targetDistanceX));

      // find the robot realtive angle of the turret
      Angle robotRelativeAngle =
          fieldRelativeAngle
              .minus(
                  robotPose
                      .getRotation()
                      .plus(new Rotation2d(robotSpeeds.omegaRadiansPerSecond * 0.02)))
              .getMeasure();

      // find the angle of the hood from the predicted pose
      //Angle estimatedHoodAngle = getHoodAngle(totalDistance);
      // TODO: remove the line below since its for debug
      m_flyWheel.setSpeed(ballSpeed);

      //distanceToTarget.equals(getDistance(hoodAngle, ballSpeed));

      return new ShootingSolution(hoodAngle, robotRelativeAngle, ballSpeed);
    } else {
      SmartDashboard.putBoolean("Valid Shooting Solution", false);
      return new ShootingSolution(Degrees.of(0), Degrees.of(0),MetersPerSecond.of(0));
    }
  }

  @Override
  public void periodic() {
    m_currentShootingSolution =
        getShootingSolution(
            m_drivetrain::getPose2d, m_drivetrain::getSpeeds, m_zone::getGoalPose);

    SmartDashboard.putBoolean("Can Shoot", canShootFuel());
    //ShootingMechanismConstants.ballVelocity = MetersPerSecond.of(SmartDashboard.getNumber("Ball Velocity", 7.5));
  }

  private boolean canShootFuel() {

    GoalPose goalPose = m_zone.getGoalPose();
    boolean check =
        m_turret
                .getPosition()
                .isNear(
                    m_currentShootingSolution.predictedTurretAngle,
                    ShootingMechanismConstants.turretMaximumError)
            && (!goalPose.requiresShift || ShiftData.canScore())
        //&&
        /*m_hood.getPosition()
        .isNear(m_currentShootingSolution.predictedHoodAngle,
        ShootingMechanismConstants.hoodMaximumError)*/
        //&& SmartDashboard.getBoolean("Shot Possible", false)
        && withinHoodBounds(m_currentShootingSolution.predictedHoodAngle);

    SmartDashboard.putNumber("Calculated Turret Angle (Deg)", m_currentShootingSolution.predictedTurretAngle.in(Degrees));
    SmartDashboard.putNumber("Calculated Hood Angle (Deg)", m_currentShootingSolution.predictedHoodAngle.in(Degrees));
    SmartDashboard.putNumber("Calculated FlyWheel Speed (MpS)", m_currentShootingSolution.flyWheelSpeed.in(MetersPerSecond));
    
    SmartDashboard.putNumber("Current FlyWheel Speed (MpS)", m_flyWheel.getSpeed().in(MetersPerSecond));

    // TODO: add hood
    SmartDashboard.putNumber(
        "Turret Deviation Deg",
        m_turret.getPosition().minus(m_currentShootingSolution.predictedTurretAngle).in(Degrees));
    return check;
  }
}
