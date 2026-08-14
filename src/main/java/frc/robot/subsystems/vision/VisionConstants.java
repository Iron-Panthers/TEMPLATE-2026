package frc.robot.subsystems.vision;

import static frc.robot.Constants.*;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import java.util.List;

public class VisionConstants {
  public static final double AMBIGUITY_CUTOFF = 0.1;
  public static final double Z_ERROR_CUTOFF = 0.5;

  public static final Matrix<N3, N1> VISION_STATE_STD_DEVS =
      VecBuilder.fill(0, 0, 0); // not real values, copy and psated :)

  // index 0 -> arducam-1, etc
  public static final Transform3d[] CAMERA_TRANSFORM =
      switch (getRobotType()) {
        case COMP -> new Transform3d[] {};
        case VISION -> new Transform3d[] {};
        case SIM -> new Transform3d[] {};
        default -> new Transform3d[0];
      };

  public static final List<TagCountDeviation> TAG_COUNT_DEVIATIONS =
      switch (getRobotType()) {
          // TODO: tune these?
        default -> List.of(
            // 1 tag
            new TagCountDeviation(
                new UnitDeviationParams(0, 0, 0),
                new UnitDeviationParams(0, 0, 0),
                new UnitDeviationParams(0, 0, 0)),
            // 2 tag
            new TagCountDeviation(
                new UnitDeviationParams(0, 0, 0),
                new UnitDeviationParams(0, 0, 0)),
            // 3+ tag
            new TagCountDeviation(
                new UnitDeviationParams(0, 0, 0),
                new UnitDeviationParams(0, 0, 0)));
      };

  public static final int[] IGNORE_TAGS = {};
  // public static final int[] IGNORE_TAGS = {}; // removed

  // Fixed AprilTag field layout initialization
  public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT;

  static {
    // logic for dynamically setting the april tag field layout
    AprilTagFieldLayout defaultFieldLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    List<AprilTag> aprilTags = defaultFieldLayout.getTags();
    // remove ignored tags
    aprilTags.removeIf(
        tag -> {
          for (int ignoreTag : IGNORE_TAGS) {
            if (tag.ID == ignoreTag) {
              return true;
            }
          }
          return false;
        });
    APRIL_TAG_FIELD_LAYOUT =
        new AprilTagFieldLayout(
            aprilTags, defaultFieldLayout.getFieldLength(), defaultFieldLayout.getFieldWidth());
  }

  public static record TagCountDeviation(
      UnitDeviationParams xParams, UnitDeviationParams yParams, UnitDeviationParams thetaParams) {
    protected Matrix<N3, N1> computeDeviation(double averageDistance) {
      return VecBuilder.fill(
          xParams.computeUnitDeviation(averageDistance),
          yParams.computeUnitDeviation(averageDistance),
          thetaParams.computeUnitDeviation(averageDistance));
    }

    public TagCountDeviation(UnitDeviationParams xyParams, UnitDeviationParams thetaParams) {
      this(xyParams, xyParams, thetaParams);
    }
  }

  public static record UnitDeviationParams(
      double distanceMultiplier, double eulerMultiplier, double constant) {
    private double computeUnitDeviation(double averageDistance) {
      return distanceMultiplier * averageDistance + constant;
    }
  }
}
