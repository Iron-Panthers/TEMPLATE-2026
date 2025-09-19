package frc.robot.subsystems.swerve;

import static frc.robot.subsystems.swerve.DriveConstants.DRIVE_CONFIG;
import static frc.robot.subsystems.swerve.DriveConstants.MODULE_CONSTANTS;
import static frc.robot.utility.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.swerve.DriveConstants.Gains;
import frc.robot.subsystems.swerve.DriveConstants.ModuleConfig;
import frc.robot.subsystems.swerve.DriveConstants.MotionProfileGains;
import java.util.function.Supplier;

public abstract class ModuleIOTalonFX implements ModuleIO {
  protected final TalonFX driveTalon;
  protected final TalonFX steerTalon;
  protected final CANcoder encoder;

  private final StatusSignal<Angle> drivePosition;
  private final StatusSignal<AngularVelocity> driveVelocity;
  private final StatusSignal<Voltage> driveAppliedVolts;
  private final StatusSignal<Current> driveSupplyCurrent;
  private final StatusSignal<Current> driveStatorCurrent;

  private final Supplier<Rotation2d> steerAbsolutePosition;
  private final StatusSignal<Angle> steerPosition;
  private final StatusSignal<AngularVelocity> steerVelocity;
  private final StatusSignal<Voltage> steerAppliedVolts;
  private final StatusSignal<Current> steerSupplyCurrent;
  private final StatusSignal<Current> steerStatorCurrent;

  private final TalonFXConfiguration driveConfig = new TalonFXConfiguration();
  private final TalonFXConfiguration steerConfig = new TalonFXConfiguration();
  private final CANcoderConfiguration encoderConfig = new CANcoderConfiguration();

  private final VelocityVoltage driveVelocityControl = new VelocityVoltage(0).withUpdateFreqHz(0);
  private final MotionMagicVoltage steerPositionControl =
      new MotionMagicVoltage(0).withUpdateFreqHz(0);

  public ModuleIOTalonFX(ModuleConfig config) {
    driveTalon = new TalonFX(config.driveID());
    steerTalon = new TalonFX(config.steerID());
    encoder = new CANcoder(config.encoderID());

    // Drive Config
    driveConfig.CurrentLimits.StatorCurrentLimit = 80; // TODO: Make constant
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    driveConfig.Feedback.SensorToMechanismRatio = MODULE_CONSTANTS.driveReduction();
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig.MotorOutput.Inverted =
        config.driveInverted()
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    setDriveSlot0(MODULE_CONSTANTS.driveGains());

    tryUntilOk(5, () -> driveTalon.getConfigurator().apply(driveConfig, 0.25));
    tryUntilOk(5, () -> driveTalon.setPosition(0.0, 0.25));

    // Steer Config
    steerConfig.CurrentLimits.StatorCurrentLimit = 50; // TODO: Make constant
    steerConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    steerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    steerConfig.MotorOutput.Inverted =
        config.steerInverted()
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    steerConfig.ClosedLoopGeneral.ContinuousWrap = true;
    steerConfig.Feedback.FeedbackRemoteSensorID = config.encoderID();
    steerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

    setSteerSlot0(MODULE_CONSTANTS.steerGains(), MODULE_CONSTANTS.steerMotionGains());

    tryUntilOk(5, () -> steerTalon.getConfigurator().apply(steerConfig, 0.25));

    // Encoder Config
    encoderConfig.MagnetSensor.MagnetOffset = -config.absoluteEncoderOffset().getRotations();
    encoder.getConfigurator().apply(encoderConfig);

    // canbus optimization TODO: Explain what Canbus optimization is
    drivePosition = driveTalon.getPosition();
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveSupplyCurrent = driveTalon.getSupplyCurrent();
    driveStatorCurrent = driveTalon.getStatorCurrent();

    steerAbsolutePosition =
        () -> Rotation2d.fromRotations(encoder.getAbsolutePosition().getValueAsDouble());
    steerPosition = steerTalon.getPosition();
    steerVelocity = steerTalon.getVelocity();
    steerAppliedVolts = steerTalon.getMotorVoltage();
    steerSupplyCurrent = steerTalon.getSupplyCurrent();
    steerStatorCurrent = steerTalon.getStatorCurrent();
    // TODO: Why is this 100 Hz when everything else is 50 Hz? (Do we use can FD?)
    BaseStatusSignal.setUpdateFrequencyForAll(
        100,
        drivePosition,
        driveVelocity,
        driveAppliedVolts,
        driveSupplyCurrent,
        driveStatorCurrent,
        encoder.getAbsolutePosition(),
        steerPosition,
        steerVelocity,
        steerAppliedVolts,
        steerSupplyCurrent,
        steerStatorCurrent);

    driveTalon.optimizeBusUtilization();
    steerTalon.optimizeBusUtilization();
    encoder.optimizeBusUtilization();

    driveTalon.setPosition(0, 1.0);
    steerTalon.setPosition(steerAbsolutePosition.get().getRotations(), 1.0);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.driveMotorConnected =
        BaseStatusSignal.refreshAll(
                drivePosition,
                driveVelocity,
                driveAppliedVolts,
                driveSupplyCurrent,
                driveStatorCurrent)
            .isOK();
    inputs.drivePositionRads = Units.rotationsToRadians(drivePosition.getValueAsDouble());
    inputs.drivePositionMeters =
        Units.rotationsToRadians(drivePosition.getValueAsDouble()) * DRIVE_CONFIG.wheelRadius();
    inputs.driveVelocityRadsPerSec = Units.rotationsToRadians(driveVelocity.getValueAsDouble());
    inputs.driveVelocityMetersPerSec =
        Units.rotationsToRadians(driveVelocity.getValueAsDouble()) * DRIVE_CONFIG.wheelRadius();
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveSupplyCurrent = driveSupplyCurrent.getValueAsDouble();
    inputs.driveStatorCurrent = driveStatorCurrent.getValueAsDouble();

    inputs.steerMotorConnected =
        BaseStatusSignal.refreshAll(
                steerPosition,
                steerVelocity,
                steerAppliedVolts,
                steerSupplyCurrent,
                steerStatorCurrent)
            .isOK();
    inputs.steerAbsolutePostion = steerAbsolutePosition.get();
    inputs.steerPosition = Rotation2d.fromRotations(steerPosition.getValueAsDouble());
    inputs.steerVelocityRadsPerSec = Units.rotationsToRadians(steerVelocity.getValueAsDouble());
    inputs.steerAppliedVolts = steerAppliedVolts.getValueAsDouble();
    inputs.steerSupplyCurrent = steerSupplyCurrent.getValueAsDouble();
    inputs.steerStatorCurrent = steerStatorCurrent.getValueAsDouble();
  }

  @Override
  public void runDriveVelocitySetpoint(double velocityRadsPerSec) {
    driveTalon.setControl(
        driveVelocityControl.withVelocity(Units.radiansToRotations(velocityRadsPerSec)));
  }

  @Override
  public void runSteerPositionSetpoint(double angleRads) {
    steerTalon.setControl(steerPositionControl.withPosition(Units.radiansToRotations(angleRads)));
  }

  @Override
  public void setDriveSlot0(Gains gains) {
    driveConfig.Slot0.kP = gains.kP();
    driveConfig.Slot0.kI = gains.kI();
    driveConfig.Slot0.kD = gains.kD();
    driveConfig.Slot0.kS = gains.kS();
    driveConfig.Slot0.kV = gains.kV();
    driveConfig.Slot0.kA = gains.kA();
    driveTalon.getConfigurator().apply(driveConfig);
  }

  @Override
  public void setSteerSlot0(Gains gains, MotionProfileGains motionProfileGains) {
    steerConfig.Slot0.kP = gains.kP();
    steerConfig.Slot0.kI = gains.kI();
    steerConfig.Slot0.kD = gains.kD();
    steerConfig.Slot0.kS = gains.kS();
    steerConfig.Slot0.kV = gains.kV();
    steerConfig.Slot0.kA = gains.kA();
    steerConfig.MotionMagic.MotionMagicCruiseVelocity = motionProfileGains.cruiseVelocity();
    steerConfig.MotionMagic.MotionMagicAcceleration = motionProfileGains.acceleration();
    steerConfig.MotionMagic.MotionMagicJerk = motionProfileGains.jerk();
    steerTalon.getConfigurator().apply(steerConfig);
  }
}
