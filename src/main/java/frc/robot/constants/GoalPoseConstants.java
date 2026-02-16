package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.FMS.Zones.Zone;

public class GoalPoseConstants {
    public static class GoalPose {
        public final Pose2d pose;
        public final boolean requiresShift;
        public final Zone[] shootableZones;

        public GoalPose(Pose2d pose, boolean requiresShift, Zone... shootableZones) {
            this.pose = pose;
            this.requiresShift = requiresShift;
            this.shootableZones = shootableZones;
        }

        /**
         * Checks the provided zone to see if its included within the shootable zones array
         * @param zone The zone you are attempting to shoot from
         * @return True if you can shoot from the provided zone
         */
        public boolean canShootInZone(Zone zone) {
            for (Zone shootableZone : shootableZones) {
                if (shootableZone.equals(zone)) return true;
            }
            return false;
        }
    }
    
    public static final GoalPose BLUE_HUB = new GoalPose(WaypointConstants.blueHub,true,Zone.ALLIANCE_ZONE);
    public static final GoalPose RED_HUB = new GoalPose(WaypointConstants.redHub,true,Zone.ALLIANCE_ZONE);
}
