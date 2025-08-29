package frc.robot.subsystems.superstructure.tongue;

import frc.robot.Constants;
import java.util.Optional;

public class TongueConstants {
  public static final TongueConfig TONGUE_CONFIG =
      switch (Constants.getRobotType()) {
        case COMP -> new TongueConfig(7, 0);
        case ALPHA -> new TongueConfig(15, 0);
        case PROG -> new TongueConfig(0, 0);
        case SIM -> new TongueConfig(0, 0);
      };

  public record TongueConfig(int servoID, int servoSensorID) {}

  public static final boolean INVERT_MOTOR = true;

  public static final double POSITION_TARGET_EPSILON = 5;
  public static final double TONGUE_OFFSET = 0;
  // SOFT LIMITS
  public static final Optional<Double> UPPER_EXTENSION_LIMIT =
      Optional.empty(); // top limit is 121 rotations
  public static final Optional<Double> LOWER_EXTENSION_LIMIT =
      Optional.empty(); // top limit is 121 rotations

  // top limit is 121 rotations

  // CURRENT LIMITS
  public static final double UPPER_VOLT_LIMIT = 3;
  public static final double LOWER_VOLT_LIMIT = -3;
  public static final double SUPPLY_CURRENT_LIMIT = 30;
}
