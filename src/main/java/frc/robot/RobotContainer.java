
package frc.robot;


import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.commands.AutoCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.RollerCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.RollerSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


public class RobotContainer {

  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final RollerSubsystem rollerSubsystem = new RollerSubsystem();

  private final CommandPS4Controller driverController = new CommandPS4Controller(
      0);

  private final CommandPS4Controller operatorController = new CommandPS4Controller(
      1);

  private final SendableChooser<CommandPS4Controller> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    configureBindings();
    autoChooser.setDefaultOption("Autonomous", driverController);
  }

 
  private void configureBindings() {



    driveSubsystem.setDefaultCommand(new DriveCommand(
        () -> -driverController.getLeftY(),
        () -> -driverController.getRightX(),
        driveSubsystem));

    rollerSubsystem.setDefaultCommand(new RollerCommand(
        () -> operatorController.getR2Axis(),
        () -> operatorController.getL2Axis(),
        rollerSubsystem));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
