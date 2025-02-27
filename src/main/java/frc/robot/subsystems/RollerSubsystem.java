
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;


public class RollerSubsystem extends SubsystemBase {
  private final SparkMax rollerMotor;

  public RollerSubsystem() {

    rollerMotor = new SparkMax(5, MotorType.kBrushed);


    rollerMotor.setCANTimeout(250);

    SparkMaxConfig rollerConfig = new SparkMaxConfig();
    rollerConfig.voltageCompensation(10);
    rollerConfig.smartCurrentLimit(60);
    rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
  }


  public void runRoller(double forward, double reverse) {
    rollerMotor.set(forward - reverse);
  }
}
