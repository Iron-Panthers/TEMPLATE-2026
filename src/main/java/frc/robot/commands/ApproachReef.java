package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotState;
import frc.robot.subsystems.swerve.Drive;
import java.util.function.Supplier;

// wrapper for autoalign FollowPathCommand
public class ApproachReef extends SequentialCommandGroup {

  private double iteration = 0;

  public enum LevelOffsets {
    // metres
    L4_OFFSET(0.14),
    L3_OFFSET(0.07),
    PREP_L4_OFFSET(0.5),
    L2_OFFSET(0),
    L1_OFFSET(0.04);
    public double levelOffset;

    private LevelOffsets(double levelOffset) {
      this.levelOffset = levelOffset;
    }

    public double getLevelOffset() {
      return levelOffset;
    }
  }

  public class ReefAlign extends Command {
    private final Supplier<LevelOffsets> levelOffsetSupplier;
    private final boolean bSide;
    private Command reefAlign;
    private LevelOffsets pastLevelOffset;

    public ReefAlign(Supplier<LevelOffsets> levelOffsetSupplier, boolean bSide) {
      this.levelOffsetSupplier = levelOffsetSupplier;
      this.bSide = bSide;
      pastLevelOffset = levelOffsetSupplier.get();
    }

    @Override
    public void initialize() {
      calculatePath();
    }

    @Override
    public void execute() {
      if (reefAlign != null) {
        reefAlign.execute();
      }
      if ((iteration > 20 && !(RobotState.getInstance().alignError() < 0.5)
              || (RobotState.getInstance().alignError() < 2
                  && levelOffsetSupplier.get() == LevelOffsets.PREP_L4_OFFSET))
          || pastLevelOffset != levelOffsetSupplier.get()) {
        calculatePath();
        iteration = 0;
      }
      iteration++;
      pastLevelOffset = levelOffsetSupplier.get();
    }

    @Override
    public boolean isFinished() {
      return reefAlign == null
          ? false
          : reefAlign.isFinished()
              && (RobotState.getInstance().alignError() < 0.5
                  || (RobotState.getInstance().alignError() < 2
                      && levelOffsetSupplier.get() == LevelOffsets.PREP_L4_OFFSET))
              && Math.abs(RobotState.getInstance().getVelocity().getNorm()) < 0.1;
    }

    @Override
    public void end(boolean interrupted) {
      if (reefAlign != null) {
        reefAlign.end(interrupted);
      }
      reefAlign = null;
    }

    public void calculatePath() {
      try {
        reefAlign =
            RobotState.getInstance()
                .approachReefCommand(
                    levelOffsetSupplier.get().getLevelOffset(),
                    bSide,
                    levelOffsetSupplier.get() == LevelOffsets.L1_OFFSET ? true : false);
        reefAlign.initialize();
      } catch (Exception e) {
        e.printStackTrace();
        if (Math.abs(RobotState.getInstance().getVelocity().getNorm()) < 0.1) {
          end(true);
        }
        System.out.println("Already at target.");
      }
    }
  }

  public ApproachReef(Supplier<LevelOffsets> levelOffsetSupplier, boolean bSide, Drive drive) {
    addCommands(new VelocityClamp(drive), new ReefAlign(levelOffsetSupplier, bSide));
  }
}
