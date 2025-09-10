package frc.robot.subsystems.vision;

import static frc.robot.Constants.*;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.List;

public class VisionConstants {
  public static final double AMBIGUITY_CUTOFF = 0.1;
  public static final double Z_ERROR_CUTOFF = 0.5;

  // index 0 -> arducam-1, etc
  public static final Transform3d[] CAMERA_TRANSFORM =
      switch (getRobotType()) {
        case COMP -> new Transform3d[] {
          // arducam-1 (front left)
          new Transform3d(
              0.299, 0.2744, 0.3464, new Rotation3d(0, -Math.toRadians(35), Math.toRadians(55))),
          // arducam-2 (front center)
          new Transform3d(0.3017, 0, 0.3373, new Rotation3d(0, -Math.toRadians(35), 0)),
          // arducam-3 (front right)
          new Transform3d(
              0.299, -0.2744, 0.3464, new Rotation3d(0, -Math.toRadians(35), -Math.toRadians(55))),
          // arducam-4 (back right)
          new Transform3d(
              -0.17, -0.298, 0.3651, new Rotation3d(0, 0, Math.PI - Math.toRadians(12))),
          // arducam-5 (back left)
          new Transform3d(-0.17, 0.298, 0.3651, new Rotation3d(0, 0, -Math.PI + Math.toRadians(12)))
        };
        case SIM -> new Transform3d[] {
          // arducam-1 (front left)
          new Transform3d(
              0.299, 0.2744, 0.3464, new Rotation3d(0, -Math.toRadians(35), Math.toRadians(55))),
          // arducam-2 (front center)
          new Transform3d(0.3017, 0, 0.3373, new Rotation3d(0, -Math.toRadians(35), 0)),
          // arducam-3 (front right)
          new Transform3d(
              0.299, -0.2744, 0.3464, new Rotation3d(0, -Math.toRadians(35), -Math.toRadians(55))),
          // arducam-4 (back right)
          new Transform3d(
              -0.17, -0.298, 0.3651, new Rotation3d(0, 0, Math.PI - Math.toRadians(12))),
          // arducam-5 (back left)
          new Transform3d(-0.17, 0.298, 0.3651, new Rotation3d(0, 0, -Math.PI + Math.toRadians(12)))
        };
        default -> new Transform3d[0];
      };

  public static final List<TagCountDeviation> TAG_COUNT_DEVIATIONS =
      switch (getRobotType()) {
        default -> List.of(
            // 1 tag
            new TagCountDeviation(
                new UnitDeviationParams(0.2, 0.1, 0.6),
                new UnitDeviationParams(0.3, 0.1, 0.9),
                new UnitDeviationParams(0.5, 0.7, 1.5)),
            // 2 tag
            new TagCountDeviation(
                new UnitDeviationParams(0.35, 0.1, 0.4), new UnitDeviationParams(0.5, 0.7, 1.5)),
            // 3+ tag
            new TagCountDeviation(
                new UnitDeviationParams(0.25, 0.07, 0.25), new UnitDeviationParams(0.15, 1, 1.5)));
      };

  public static final int[] IGNORE_TAGS = {1, 2, 3, 4, 5, 12, 13, 14, 15, 16};
  // public static final int[] IGNORE_TAGS = {}; // removed

  // Fixed AprilTag field layout initialization
  public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT;

  static {
    AprilTagFieldLayout layout;
    try {
      // layout = // old code for loading from a file
      //     new AprilTagFieldLayout(
      //         Paths.get(
      //
      // "src/main/java/frc/robot/subsystems/vision/april_tag_layouts/reefscape_apriltags.json")
      //             .toAbsolutePath()
      //             .toString());
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
      layout =
          new AprilTagFieldLayout(
              aprilTags, defaultFieldLayout.getFieldWidth(), defaultFieldLayout.getFieldWidth());
    } catch (Exception e) {
      System.err.println("Error loading custom AprilTag field layout: " + e.getMessage());
      layout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    }
    APRIL_TAG_FIELD_LAYOUT = layout;
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
      double distanceMultiplier, double eulerMultiplier, double minimum) {
    private double computeUnitDeviation(double averageDistance) {
      return Math.max(minimum, eulerMultiplier * Math.exp(averageDistance * distanceMultiplier));
    }
  }
}
