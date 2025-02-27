package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;


public class AutoCommand extends Command {

  DriveSubsystem driveSubsystem;
  private Timer timer;
  private double seconds = 1.0;
  
  public AutoCommand(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    timer = new Timer();
    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {

    timer.restart();
  }

  @Override
  public void execute() {

    driveSubsystem.arcadeDrive(0.5, 0.0);
  }

  @Override
  public void end(boolean isInterrupted) {

    driveSubsystem.arcadeDrive(0.0, 0.0);
  }

  @Override
  public boolean isFinished() {

    return timer.get() >= seconds;
  }
}
