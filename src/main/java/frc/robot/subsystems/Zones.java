// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class Zones extends SubsystemBase{

    public enum Zone {
        ALLIANCE_ZONE(new Pose2d(0.0, 0.0,  null)),
        NEUTRAL_ZONE(new Pose2d(4.0, 2.0, null)),
        OPPONENT_ZONE(new Pose2d(7.0, 3.0, null)),
        OUT_OF_BOUNDS(null);

        private final Pose2d pose;

        Zone(Pose2d pose) {
            this.pose = pose;
        }

        public Pose2d getPose() {
            return pose;
        }
    }

    private static Zone CurrentZone = Zone.ALLIANCE_ZONE;



    public void UpdateZone(Pose2d robotPose) {
        double x = robotPose.getX();
        double y = robotPose.getY();

        if (isWithin(x, y, new Translation2d(0.0, 0.0), new Translation2d(4.0, 3.0))) {
            CurrentZone = Zone.ALLIANCE_ZONE;
        } else if (isWithin(x, y, new Translation2d(4.0, 0.0), new Translation2d(7.0, 3.0))) {
            CurrentZone = Zone.NEUTRAL_ZONE;
        } else if (isWithin(x, y, new Translation2d(7.0, 0.0), new Translation2d(10.0, 3.0))) {
            CurrentZone = Zone.OPPONENT_ZONE;
        } else {
            // Outside defined zones, handle as needed
            CurrentZone = Zone.OUT_OF_BOUNDS;
        }
    }


    private boolean isWithin(double x, double y, Translation2d corner1, Translation2d corner2) {
        double minX = Math.min(corner1.getX(), corner2.getX());
        double maxX = Math.max(corner1.getX(), corner2.getX());
        double minY = Math.min(corner1.getY(), corner2.getY());
        double maxY = Math.max(corner1.getY(), corner2.getY());

        return (x >= minX && x <= maxX) && (y >= minY && y <= maxY);

    }
    

    public void periodic() {
        SmartDashboard.putString("CurrentZone", CurrentZone.name());
    }
}
