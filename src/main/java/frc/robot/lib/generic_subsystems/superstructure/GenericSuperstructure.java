package frc.robot.lib.generic_subsystems.superstructure;

import java.util.Optional;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.Unit;

public abstract class GenericSuperstructure<G extends GenericSuperstructure.PositionTarget> {
  public interface PositionTarget {
    double getPosition(); // TODO: make this a consistant unit

    /**
     * Retrieves the tolerance value
     *
     * @return The tolerance value as a double.
     */
    double getEpsilon();
  }

  public enum ControlMode {
    POSITION,
    STOP;
  }

  private ControlMode controlMode = ControlMode.STOP;

  protected final String name;
  protected final GenericSuperstructureIO superstructureIO;

  private GenericSuperstructureIOInputsAutoLogged inputs =
      new GenericSuperstructureIOInputsAutoLogged();
  private G positionTarget;

  public GenericSuperstructure(String name, GenericSuperstructureIO superstructureIO) {
    this.name = name;
    this.superstructureIO = superstructureIO;
  }

  public void periodic() {
    // Process inputs
    superstructureIO.updateInputs(inputs);
    Logger.processInputs(name, inputs);

    // Process control mode
    switch (controlMode) {
      case POSITION -> {
        superstructureIO.runPosition(positionTarget.getPosition());
      }
      case STOP -> {
        superstructureIO.stop();
      }
    }

    Logger.recordOutput("Superstructure/" + name + "/Target", positionTarget.toString());
    Logger.recordOutput("Superstructure/" + name + "/Control Mode", controlMode.toString());
    Logger.recordOutput("Superstructure/" + name + "/Reached target", reachedTarget());
    Logger.recordOutput(
        "Superstructure/" + name + "/Target Position", positionTarget.getPosition());
  }

  public G getPositionTarget() {
    return positionTarget;
  }

  public void setPositionTarget(G positionTarget) {
    setControlMode(ControlMode.POSITION);
    this.positionTarget = positionTarget;
  }


  public ControlMode getControlMode() {
    return controlMode;
  }

  public void setControlMode(ControlMode controlMode) {
    this.controlMode = controlMode;
  }

  /**
   * This is the zeroing function for the subsystem.
   */
  public void setOffset() {
    superstructureIO.setOffset();
  }

  public double getSupplyCurrentAmps() {
    return inputs.supplyCurrentAmps;
  }

  public double getPosition() {
    return inputs.positionRotations;
  }

  /**
   * This function returns whether or not the subsystem has reached its position target
   *
   * @return whether the subsystem has reached its position target
   */
  public boolean reachedTarget() {
    return Math.abs(inputs.positionRotations - (positionTarget.getPosition()))
        <= positionTarget.getEpsilon();
  }
}
