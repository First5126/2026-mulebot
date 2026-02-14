package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;

public class ZonesConstants {
    public static enum Zone {
        ALLIANCE_ZONE(new Pose2d(0.0, 0.0, null)),
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
}
