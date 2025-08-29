package frc.robot.subsystems.superstructure.tongue;

import static frc.robot.subsystems.superstructure.tongue.TongueConstants.TONGUE_OFFSET;

import org.littletonrobotics.junction.Logger;

public class Tongue {
  public enum TongueTarget {
    TOP(64),
    INTAKE(64),
    STOW(64),
    L1(64),
    L2(64),
    L3(40),
    L4(0),
    CLIMB(64),
    DESCORE(64);

    private double position;

    private TongueTarget(double position) {
      this.position = position;
    }

    public double getPosition() {
      return position + TONGUE_OFFSET;
    }
  }

  public enum ControlMode {
    POSITION,
    STOP,
  }

  private ControlMode controlMode = ControlMode.STOP;

  private final TongueIO io;

  private TongueIOInputsAutoLogged inputs = new TongueIOInputsAutoLogged();
  private TongueTarget positionTarget;

  public Tongue(TongueIO io) {
    this.io = io;

    setPositionTarget(TongueTarget.TOP);
    setControlMode(ControlMode.STOP);
  }

  public void periodic() {
    // Process inputs
    io.updateInputs(inputs);
    Logger.processInputs("Tongue", inputs);

    // Process control mode
    switch (controlMode) {
      case POSITION -> {
        io.runPosition(positionTarget.getPosition());
      }
      case STOP -> {
        io.stop();
      }
    }
    Logger.recordOutput("Tongue/Target", positionTarget.toString());
    Logger.recordOutput("Tongue/Control Mode", controlMode.toString());
    Logger.recordOutput("Tongue/Reached target", reachedTarget());

    Logger.recordOutput("Superstructure/" + "Tongue" + "/Target", positionTarget.toString());
    Logger.recordOutput("Superstructure/" + "Tongue" + "/Control Mode", controlMode.toString());
    Logger.recordOutput("Superstructure/" + "Tongue" + "/Reached target", reachedTarget());
  }

  public TongueTarget getPositionTarget() {
    return positionTarget;
  }

  public void setPositionTarget(TongueTarget positionTarget) {
    setControlMode(ControlMode.POSITION);
    this.positionTarget = positionTarget;
  }

  public ControlMode getControlMode() {
    return controlMode;
  }

  public void setControlMode(ControlMode controlMode) {
    this.controlMode = controlMode;
  }

  public double position() {
    return inputs.angle;
  }

  public boolean reachedTarget() {
    return true;
  }

  public boolean poleDetected() {
    return inputs.pole1Detected && inputs.pole2Detected;
  }

  public boolean pole1Detected() {
    return inputs.pole1Detected;
  }

  public boolean pole2Detected() {
    return inputs.pole2Detected;
  }
}
