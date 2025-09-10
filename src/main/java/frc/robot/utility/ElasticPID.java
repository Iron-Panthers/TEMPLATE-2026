package frc.robot.utility;

import com.ctre.phoenix6.signals.GravityTypeValue;

public class ElasticPID {
  SetSlot0Lambda lambda;

  // Constructor should take in lambda for setSlot0 function
  public ElasticPID(SetSlot0Lambda setSlot0Function) {
    this.lambda = setSlot0Function;
  }
  // testtest

  // Periodic function
  public void periodic() {
    this.lambda.setSlot0(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, GravityTypeValue.Elevator_Static);
  }

  // Interface for lambda
  public interface SetSlot0Lambda {
    default void setSlot0(
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
        GravityTypeValue gravityTypeValue) {}
  }
}
