package frc.robot.lib.superstructure_template;

import static frc.robot.lib.superstructure_template.SuperstructureTempConstants.*;

import frc.robot.lib.generic_subsystems.superstructure.*;

public class SuperstructureTempIOTalonFX extends GenericSuperstructureIOTalonFX
    implements SuperstructureTempIO {

  public SuperstructureTempIOTalonFX() {
    super(
        new GenericSuperstructureConfiguration()
            .withID(SUPERSTRUCTURE_TEMP_CONFIG.motorID())
            .withMotorDirection(MOTOR_DIRECTION)
            .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT)
            .withReduction(SUPERSTRUCTURE_TEMP_CONFIG.reduction())
            .withUpperVoltageLimit(UPPER_VOLT_LIMIT)
            .withLowerVoltageLimit(LOWER_VOLT_LIMIT)
            .withCANCoderID(SUPERSTRUCTURE_TEMP_CONFIG.canCoderID())
            .withCANCoderOffset(SUPERSTRUCTURE_TEMP_CONFIG.canCoderOffset())
            .withCANCoderDirection(CANCODER_DIRECTION));

    setSlot0(
        GAINS.kP(),
        GAINS.kI(),
        GAINS.kD(),
        GAINS.kS(),
        GAINS.kV(),
        GAINS.kA(),
        GAINS.kG(),
        MOTION_MAGIC_CONFIG.acceleration(),
        MOTION_MAGIC_CONFIG.cruiseVelocity(),
        0,
        GRAVITY_TYPE);
  }

  /** Move move the arm to a position with the given degrees */
  @Override
  public void runPosition(double position) {
    super.runPosition(position / 360d); // convert degrees to rotations
  }
}
