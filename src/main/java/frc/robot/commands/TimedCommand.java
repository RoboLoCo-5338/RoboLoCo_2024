package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.Set;

public class TimedCommand extends Command {
  private long startTime = 0;
  private long time;
  private Runnable onEnd;
  private Runnable onExecute;

  public TimedCommand(long time,
                      Runnable onExecute,
                      Runnable onEnd,
                      Subsystem... requirements) {
    super();
    this.time = time;
    this.onExecute = onExecute;
    this.onEnd = onEnd;
    m_requirements = Set.of(requirements);
  }

  @Override
  public void initialize() {
    onEnd.run();
    startTime = System.currentTimeMillis();
  }

  @Override
  public void execute() {
    onExecute.run();
  }

  @Override
  public void end(boolean interrupted) {
    onEnd.run();
  }

  @Override
  public boolean isFinished() {
    return System.currentTimeMillis() - time > startTime;
  }
}
