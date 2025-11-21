package frc.robot.lib.superstructure_template;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.subsystems.canWatchdog.CANWatchdogConstants.CAN;

public class SuperstructureTempConstants {
  // TODO: Change values
  public static final SuperstructureTempConfig SUPERSTRUCTURE_TEMP_CONFIG =
      switch (Constants.getRobotType()) {
        case COMP -> new SuperstructureTempConfig(
            CAN.at(0, "insertNameHere"), CAN.at(0, "insertNameHere Encoder"), 0, 0);
        case SIM -> new SuperstructureTempConfig(
            CAN.at(0, "insertNameHere"), CAN.at(0, "insertNameHere Encoder"), 0, 0);
        default -> new SuperstructureTempConfig(0, 0, 0, 0);
      };
  // TODO: Change values
  public static final PIDGains GAINS =
      switch (Constants.getRobotType()) {
        case COMP -> new PIDGains(0, 0, 0, 0, 0, 0, 0);
        case SIM -> new PIDGains(0, 0, 0, 0, 0, 0, 0);
        default -> new PIDGains(0, 0, 0, 0, 0, 0, 0);
      };
  // TODO: Change values
  public static final MotionMagicConfig MOTION_MAGIC_CONFIG =
      switch (Constants.getRobotType()) {
        case COMP -> new MotionMagicConfig(0, 0);
        case SIM -> new MotionMagicConfig(0, 0);
        default -> new MotionMagicConfig(0, 0);
      };

  public record SuperstructureTempConfig(
      int motorID, int canCoderID, double canCoderOffset, double reduction) {}

  public record PIDGains(
      double kP, double kI, double kD, double kS, double kV, double kA, double kG) {}

  public record MotionMagicConfig(double acceleration, double cruiseVelocity) {}

  public static final GravityTypeValue GRAVITY_TYPE = GravityTypeValue.Arm_Cosine;

  public static final InvertedValue MOTOR_DIRECTION = InvertedValue.CounterClockwise_Positive;

  public static final SensorDirectionValue CANCODER_DIRECTION =
      SensorDirectionValue.Clockwise_Positive;

  public static final double POSITION_TARGET_EPSILON = 0; // TODO: Change value

  // CURRENT LIMITS
  // TODO: Change following values below
  public static final double UPPER_VOLT_LIMIT = 0;
  public static final double LOWER_VOLT_LIMIT = 0;
  public static final double SUPPLY_CURRENT_LIMIT = 0;

  // ARM POSITION CONSTANTS
  // TODO: Change values of inchesToMeters
  public static final Transform3d ELEVATOR_TO_ARM_TRANSFORM3D =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)),
          new Rotation3d(0, 0, 0));

  // PHYSICAL CONSTANTS
  public static record SuperstructureTempPhysicalConstants(
      double momentOfInertia,
      double lengthMeters,
      double minAngleRads,
      double maxAngleRads,
      boolean simulateGravity) {}

  // TODO: Change all values below
  public static final SuperstructureTempPhysicalConstants PHYSICAL_CONSTANTS =
      switch (Constants.getRobotType()) {
        case SIM -> new SuperstructureTempPhysicalConstants(0, 0, 0, 0, true);
        case COMP -> new SuperstructureTempPhysicalConstants(0, 0, 0, 0, false);
        default -> new SuperstructureTempPhysicalConstants(0, 0, 0, 0, false);
      };

  public static final double SUPERSTRUCTURETEMP_LENGTH = 0; // inches //TODO: Change value
}
