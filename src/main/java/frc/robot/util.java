package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class util {
    //takes 1 pose and then finds out bassed on your current pose and you going in a cirtain
    //dirrecton and bassed on the time it takes it will give you the calculated Pose2d
    public static Pose2d calculatePredictedPose2d(CommandSwerveDrivetrain drivetrain, double time, double speed) {
        Pose2d pose = drivetrain.getPose2d();
        double currentX = pose.getX();
        double currentY = pose.getY();

        double distance = time * speed;
        double rotation = drivetrain.getPose2d().getRotation().getRadians();

        double predictedX = currentX + distance * Math.cos(rotation);
        double predictedY = currentY + distance * Math.sin(rotation);

        return new Pose2d(
            predictedX,
            predictedY,
            new Rotation2d(rotation)
        );

    }
}
