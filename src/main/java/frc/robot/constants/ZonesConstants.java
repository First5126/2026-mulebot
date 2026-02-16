package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class ZonesConstants {
    public static enum Zone {
        ALLIANCE_ZONE(new Translation2d(0.0, 0.0), new Translation2d(4.0, 8.0)),
        NEUTRAL_ZONE(new Translation2d(4.0, 0.0), new Translation2d(7.0, 8.0)),
        OPPONENT_ZONE(new Translation2d(7.0, 0.0), new Translation2d(10.0, 8.0)),
        OUT_OF_BOUNDS(null, null);

        private final Translation2d topLeftTranslation;
        private final Translation2d bottomRightTranslation;

        Zone(Translation2d topLeftTranslation,Translation2d bottomRightTranslation) {
            this.topLeftTranslation = topLeftTranslation;
            this.bottomRightTranslation = bottomRightTranslation;
        }

        public Translation2d getTopLeftTranslation() {
            return topLeftTranslation;
        }

        public Translation2d getBottomRightTranslation() {
            return bottomRightTranslation;
        }
  }
}
