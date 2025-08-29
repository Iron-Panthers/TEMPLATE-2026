package frc.robot.subsystems.swerve.controllers;

import static frc.robot.subsystems.swerve.DriveConstants.HEADING_CONTROLLER_CONSTANTS;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;

public class HeadingController {
  private ProfiledPIDController controller;
  private Supplier<Rotation2d> headingSupplier;

  private Rotation2d targetHeading;

  public HeadingController(Supplier<Rotation2d> headingSupplier, Rotation2d targetHeading) {
    this.headingSupplier = headingSupplier;
    this.targetHeading = targetHeading;

    controller =
        new ProfiledPIDController(
            HEADING_CONTROLLER_CONSTANTS.kP(),
            0,
            HEADING_CONTROLLER_CONSTANTS.kD(),
            new Constraints(
                HEADING_CONTROLLER_CONSTANTS.maxVelocity(),
                HEADING_CONTROLLER_CONSTANTS.maxAcceleration()),
            Constants.PERIODIC_LOOP_SEC);
    controller.setTolerance(Units.degreesToRadians(HEADING_CONTROLLER_CONSTANTS.tolerance()));
    controller.enableContinuousInput(-Math.PI, Math.PI);
    controller.reset(headingSupplier.get().getRadians());
  }

  public double update() {
    double output =
        controller.calculate(headingSupplier.get().getRadians(), targetHeading.getRadians());

    return Math.abs(output) > 0.02 ? output : 0; // To prevent jittering
  }

  @AutoLogOutput(key = "Swerve/AtTarget")
  public boolean atTarget() {
    return epsilonEquals(
        controller.getSetpoint().position,
        controller.getGoal().position,
        Units.degreesToRadians(HEADING_CONTROLLER_CONSTANTS.tolerance()));
  }

  public boolean epsilonEquals(double a, double b, double epsilon) {
    return (a - epsilon <= b) && (a + epsilon >= b);
  }

  public Rotation2d getTargetHeading() {
    return targetHeading;
  }

  public void setTargeHeading(Rotation2d targetHeading) {
    this.targetHeading = targetHeading;
  }
}
