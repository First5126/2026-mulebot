package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.HoodConstants;
import frc.robot.constants.TurretConstants;

public class ShootingMechanism extends SubsystemBase {
    public static class ShootingSolution {
        public Angle predictedHoodAngle;
        public Angle predictedTurretAngle;

        public ShootingSolution(Angle hoodAngle, Angle turretAngle) {
            predictedHoodAngle = hoodAngle;
            predictedTurretAngle = turretAngle;
        }
    }

    public ShootingMechanism() {}

    /**
     * @param robotPoseSupplier The current pose of the robot
     * @param speed The chassis speeds of the drivetrain
     * @param airTimeSupplier The time it takes for a ball to land at the target from our current pose
     * @param targetPoseSupplier The pose of the target
     * @return A shooting soltuion {@link frc.robot.subsystems.ShootingMechanism.ShootingSolution} that contains the predicted angle for the hood and turret
     */
    public ShootingSolution geShootingSolution(Supplier<Pose2d> robotPoseSupplier, 
    Supplier<ChassisSpeeds> speed, Supplier<Pose2d> targetPoseSupplier) {

        // check to see if our suppliers are valid
        if (robotPoseSupplier.get() != null && targetPoseSupplier.get() != null && speed.get() != null) {
            SmartDashboard.putBoolean("Valid Shooting Solution", true);

            // retreive the value of all the suppliers
            Pose2d robotPose = robotPoseSupplier.get();
            ChassisSpeeds robotSpeeds = speed.get();
            Pose2d targetPose = targetPoseSupplier.get();

            // find air time from distance
            double distanceToTarget = robotPose.getTranslation().getDistance(targetPose.getTranslation());
            double airTime = TurretConstants.DISTANCE_TO_TIME_INTERPOLATOR.get(distanceToTarget);

            // find how far we travel by the time the ball will reach the target
            double predicatedDistance = airTime * Math.hypot(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond);

            // find the angle of the the speeds that are currently in robotcentric
            Rotation2d rotation = new Rotation2d(Math.atan2(robotSpeeds.vyMetersPerSecond,robotSpeeds.vxMetersPerSecond));

            // add the angle of the speeds to get the field centric velocity angle
            rotation = rotation.plus(robotPose.getRotation());

            // find the predicated x and y of our robot pose
            double predictedX = robotPose.getX() + predicatedDistance * Math.cos(rotation.getRadians());
            double predictedY = robotPose.getY() + predicatedDistance * Math.sin(rotation.getRadians());
            
            // get the turret pose
            Pose2d turretPose = new Pose2d(predictedX,predictedY,new Rotation2d()).plus(TurretConstants.TURRET_OFFSET);

            // find the distance to target from predicted pose
            double targetDistanceX = targetPose.getX() - turretPose.getX();
            double targetDistanceY = targetPose.getY() - turretPose.getY();

            // find the field relative angle from the distance
            Rotation2d fieldRelativeAngle =
                Rotation2d.fromRadians(Math.atan2(targetDistanceY, targetDistanceX));

            // find the robot realtive angle of the turret
            Angle robotRelativeAngle = fieldRelativeAngle.minus(robotPose.getRotation()).getMeasure();

            // find the angle of the hood from the predicted pose
            Angle hoodAngle = Rotations.of(HoodConstants.DISTANCE_TO_ANGLE_INTERPOLATOR.get(Math.hypot(targetDistanceX, targetDistanceY)));

            return new ShootingSolution(hoodAngle, robotRelativeAngle);
        }
        else {
            SmartDashboard.putBoolean("Valid Shooting Solution", false);
            return new ShootingSolution(Degrees.of(0),Degrees.of(0));
        } 
    }
}
