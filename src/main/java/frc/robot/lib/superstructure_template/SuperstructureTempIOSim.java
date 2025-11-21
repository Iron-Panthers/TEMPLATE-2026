package frc.robot.lib.superstructure_template;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.lib.generic_subsystems.superstructure.*;

public class SuperstructureTempIOSim extends GenericSuperstructureIOSim
    implements SuperstructureTempIO {

  //  private final SuperstructureTempMechanismSim superstructureTempSim;
  private final double reduction;

  public SuperstructureTempIOSim() {
    super(SuperstructureTempConstants.SUPERSTRUCTURE_TEMP_CONFIG.motorID());

    this.reduction = SuperstructureTempConstants.SUPERSTRUCTURE_TEMP_CONFIG.reduction();
    // TODO: Uncomment
    // superstructureTempSim =
    //     new SuperstructureTempMechanismSim(
    //         DCMotor.getKrakenX60Foc(1) //TODO: Possibly change value
    //         reduction,
    //         SuperstructureTempConstants.PHYSICAL_CONSTANTS.momentOfInertia(),
    //         SuperstructureTempConstants.PHYSICAL_CONSTANTS.lengthMeters(),
    //         SuperstructureTempConstants.PHYSICAL_CONSTANTS.minAngleRads(),
    //         SuperstructureTempConstants.PHYSICAL_CONSTANTS.maxAngleRads(),
    //         SuperstructureTempConstants.PHYSICAL_CONSTANTS.simulateGravity(),
    //         0);
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

  @Override
  public void updateInputs(GenericSuperstructureIOInputs inputs) {
    // Update TalonFX state
    talon.getSimState().setSupplyVoltage(16);

    double appliedVoltage = talon.getSimState().getMotorVoltage();
    // TODO: Uncomment everything until single slash
    // Simulate physics
    //   superstructureTempSim.setInputVoltage(appliedVoltage);
    //   superstructureTempSim.update(0.02); //Don't change this

    // Convert position and velocity from meters to rotations for the TalonFX sensor
    //   double rotations = superstructureTempSim.getAngleRads() / (2 * Math.PI * reduction);
    //   double velocityRPS = superstructureTempSim.getVelocityRadPerSec() / (2 * Math.PI *
    // reduction);

    //   talon.getSimState().setRawRotorPosition(rotations);
    //   talon.getSimState().setRotorVelocity(velocityRPS);

    //   inputs.isConnected = true;
    //   inputs.positionRotations = rotations;
    //   inputs.velocityRotPerSec = velocityRPS;
    //   inputs.appliedVolts = appliedVoltage;
    //   inputs.supplyCurrentAmps = 1.0; // Not simulated
    //   inputs.tempCelsius = 25.0; // Not simulated
  }

  // @Override
  // public void setOffset() {
  //   superstructureTempSim.setState(0, 0);
  // }

  /** Move move the arm to a position with the given degrees */
  @Override
  public void runPosition(double position) {
    super.runPosition(position / 360d); // convert degrees to rotations
  }
}
