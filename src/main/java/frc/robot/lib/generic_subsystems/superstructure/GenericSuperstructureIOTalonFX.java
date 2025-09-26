package frc.robot.lib.generic_subsystems.superstructure;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public abstract class GenericSuperstructureIOTalonFX implements GenericSuperstructureIO {
  // Talon FX Motor
  protected final TalonFX talon;

  // Motor config
  protected final TalonFXConfiguration config = new TalonFXConfiguration();

  // status signals
  private final StatusSignal<Angle> positionRotations;
  private final StatusSignal<AngularVelocity> velocityRPS;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Temperature> temp;

  // zeroing stuff
  private final double zeroingVolts;
  private final double zeroingOffset;

  protected final VoltageOut voltageOutput = new VoltageOut(0).withUpdateFreqHz(0);
  private final NeutralOut neutralOutput = new NeutralOut();
  private final MotionMagicVoltage positionControl = new MotionMagicVoltage(0).withUpdateFreqHz(0);

  /** Constructs a new GenericSuperstructureIOTalonFX. */
  public GenericSuperstructureIOTalonFX(GenericSuperstructureConfiguration superstructureConfig) {
    /* set the zeroing values such that when the robot zeros it will apply the
     * zeroing volts and when it reaches a resistance from part of the mechanism, it
     * sets the position to the zeroing offset */
    this.zeroingVolts = superstructureConfig.zeroingVolts;
    this.zeroingOffset = superstructureConfig.zeroingOffset;

    // VOLTAGE, LIMITS AND RATIO CONFIG
    config.MotorOutput.Inverted = superstructureConfig.motorDirection;
    config.CurrentLimits.SupplyCurrentLimit = superstructureConfig.supplyCurrentLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.Voltage.withPeakForwardVoltage(superstructureConfig.upperVoltLimit);
    config.Voltage.withPeakReverseVoltage(superstructureConfig.lowerExtensionLimit);
    config.Feedback.withSensorToMechanismRatio(superstructureConfig.reduction);

    config.SoftwareLimitSwitch.withReverseSoftLimitEnable(true);
    config.SoftwareLimitSwitch.withReverseSoftLimitThreshold(
        superstructureConfig.lowerExtensionLimit);
    config.SoftwareLimitSwitch.withReverseSoftLimitEnable(true);
    config.SoftwareLimitSwitch.withReverseSoftLimitThreshold(
        superstructureConfig.upperExtensionLimit);

    talon = new TalonFX(superstructureConfig.id);

    if (superstructureConfig.canCoderID != -1) { // TODO: Make default -1 or use Optional
      CANcoder canCoder = new CANcoder(superstructureConfig.canCoderID);
      canCoder
          .getConfigurator()
          .apply(
              new CANcoderConfiguration()
                  .withMagnetSensor(
                      new MagnetSensorConfigs()
                          .withAbsoluteSensorDiscontinuityPoint(0.5)
                          .withSensorDirection(superstructureConfig.canCoderDirection)
                          .withMagnetOffset(superstructureConfig.canCoderOffset)));
      config.Feedback.withRemoteCANcoder(canCoder);
    }
    // TODO: Do we need to try until OK?
    talon.getConfigurator().apply(config);
    setOffset();
    talon.setNeutralMode(NeutralModeValue.Brake);

    // STATUS SIGNALS
    velocityRPS = talon.getVelocity();
    appliedVolts = talon.getMotorVoltage();
    supplyCurrent = talon.getSupplyCurrent();
    temp = talon.getDeviceTemp();
    positionRotations = talon.getPosition();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50, positionRotations, velocityRPS, appliedVolts, supplyCurrent, temp);
  }

  @Override
  public void updateInputs(GenericSuperstructureIOInputs inputs) {
    inputs.isConnected =
        BaseStatusSignal.refreshAll(
                positionRotations, velocityRPS, appliedVolts, supplyCurrent, temp)
            .isOK();
    inputs.positionRotations = positionRotations.getValueAsDouble();
    inputs.velocityRotPerSec = velocityRPS.getValueAsDouble();
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.tempCelsius = temp.getValueAsDouble();
  }

  @Override
  public void runPosition(double rotations) {
    talon.setControl(positionControl.withPosition(rotations));
  }

  @Override
  public void runCharacterization() {
    talon.setControl(voltageOutput.withOutput(zeroingVolts));
  }

  @Override
  public void stop() {
    talon.setControl(neutralOutput);
  }

  @Override
  public void setOffset() {
    talon.getConfigurator().setPosition(zeroingOffset);
  }

  /**
   * Sets all of the PID and motion magic gains.
   *
   * @param kP Proportional gain
   * @param kI Integral gain
   * @param kD Derivative gain
   * @param kS Static gain
   * @param kV Velocity gain
   * @param kA Acceleration gain
   * @param kG Gravity gain
   * @param motionMagicAcceleration Motion magic acceleration (rotations per second squared)
   * @param motionMagicCruiseVelocity Motion magic cruise velocity (rotations per second)
   * @param motionMagicJerk Motion magic jerk (rotations per second cubed)
   * @param gravityTypeValue Gravity compensation type
   */
  @Override
  public void setSlot0(
      double kP,
      double kI,
      double kD,
      double kS,
      double kV,
      double kA,
      double kG,
      double motionMagicAcceleration,
      double motionMagicCruiseVelocity,
      double motionMagicJerk,
      GravityTypeValue gravityTypeValue) {
    Slot0Configs gainsConfig = new Slot0Configs();
    gainsConfig.kP = kP;
    gainsConfig.kI = kI;
    gainsConfig.kD = kD;
    gainsConfig.kS = kS;
    gainsConfig.kV = kV;
    gainsConfig.kA = kA;
    gainsConfig.kG = kG;
    gainsConfig.GravityType = gravityTypeValue;

    MotionMagicConfigs motionMagicConfig = new MotionMagicConfigs();
    motionMagicConfig.MotionMagicAcceleration = motionMagicAcceleration;
    motionMagicConfig.MotionMagicCruiseVelocity = motionMagicCruiseVelocity;
    motionMagicConfig.MotionMagicJerk = motionMagicJerk;

    talon.getConfigurator().apply(gainsConfig);
    talon.getConfigurator().apply(motionMagicConfig);
  }
}
