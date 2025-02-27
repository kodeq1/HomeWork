
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import java.util.function.DoubleSupplier;

public class DriveCommand extends Command {
  private final DoubleSupplier xSpeed;
  private final DoubleSupplier zRotation;
  private final DriveSubsystem driveSubsystem;

  public DriveCommand(
      DoubleSupplier xSpeed, DoubleSupplier zRotation, DriveSubsystem driveSubsystem) {

    this.xSpeed = xSpeed;
    this.zRotation = zRotation;
    this.driveSubsystem = driveSubsystem;

    addRequirements(this.driveSubsystem);
  }

  @Override
  public void initialize() {
  }
  @Override
  public void execute() {
    driveSubsystem.arcadeDrive(xSpeed.getAsDouble(), zRotation.getAsDouble());
  }

  @Override
  public void end(boolean isInterrupted) {
  }

  @Override
  public boolean isFinished() {

    return false;
  }
}
