package frc.robot.lib.superstructure_template;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.lib.generic_subsystems.superstructure.*;

public class SuperstructureTempIOSim extends GenericSuperstructureIOSim implements SuperstructureTempIO {
 
 private final SingleJointedArmSim SuperstructureTempSim;
 private final double reduction;

  public SuperstructureTempIOSim() {
    super(SuperstructureTempConstants.SUPERSTRUCTURE_TEMP_CONFIG.motorID());

    this.reduction = SuperstructureTempConstants.SUPERSTRUCTURE_TEMP_CONFIG.reduction();

    SuperstructureTempSim =
        new SingleJointedArmSim(
            DCMotor.getKrakenX60Foc(1) //TODO: Possibly change value
            reduction,
            SuperstructureTempConstants.PHYSICAL_CONSTANTS.momentOfInertia(),
            SuperstructureTempConstants.PHYSICAL_CONSTANTS.lengthMeters(),
            SuperstructureTempConstants.PHYSICAL_CONSTANTS.minAngleRads(),
            SuperstructureTempConstants.PHYSICAL_CONSTANTS.maxAngleRads(),
            SuperstructureTempConstants.PHYSICAL_CONSTANTS.simulateGravity(),
            0);
    setOffset();
    setSlot0(
        SuperstructureTempConstants.GAINS.kP(),
        SuperstructureTempConstants.GAINS.kI(),
        SuperstructureTempConstants.GAINS.kD(),
        SuperstructureTempConstants.GAINS.kS(),
        SuperstructureTempConstants.GAINS.kV(),
        SuperstructureTempConstants.GAINS.kA(),
        SuperstructureTempConstants.GAINS.kG(),
        SuperstructureTempConstants.MOTION_MAGIC_CONFIG.acceleration(),
        SuperstructureTempConstants.MOTION_MAGIC_CONFIG.cruiseVelocity(),
        0,
        SuperstructureTempConstants.GRAVITY_TYPE);
    }
}
