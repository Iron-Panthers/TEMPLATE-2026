package frc.robot.lib.superstructure_template;

import frc.robot.lib.generic_subsystems.superstructure.*;
import org.littletonrobotics.junction.Logger;

public class SuperstructureTemp extends GenericSuperstructure<SuperstructureTemp.SuperstructureTempTarget> {
    public enum SuperstructureTempTarget implements GenericSuperstructure.PositionTarget {
        TOP(0); //TODO: put actual values
  
      private double position;
      private static final double EPSILON = SuperstructureTempConstants.POSITION_TARGET_EPSILON;
  
      private SuperstructureTempTarget(double position) {
        this.position = position;
      }
  
      public double getPosition() {
        return position;
      }
  
      @Override
      public double getEpsilon() {
        return EPSILON;
      }
    }

    public SuperstructureTemp(SuperstructureTempIO io) {
      super("SuperstructureTemp", io);
      setPositionTarget(SuperstructureTempTarget.TOP);
      setControlMode(ControlMode.STOP);
    }
  
    @Override
    public void periodic() {
      super.periodic();
  
      Logger.recordOutput(
          "Superstructure/Arm/PositionTargetRotations", getPositionTarget().getPosition() / 360d);
    }
  
    /**
     * This function returns whether or not the subsystem has reached its position target
     *
     * @return whether the subsystem has reached its position target
     */
    public boolean reachedTarget() {
      return Math.abs(super.getPosition() - (super.getPositionTarget().getPosition() / 360d))
          <= super.getPositionTarget().getEpsilon();
    }
  
    /** Returns the position of the arm in DEGREES */
    public double getPosition() {
      return super.getPosition() * 360.0;
    }
  }