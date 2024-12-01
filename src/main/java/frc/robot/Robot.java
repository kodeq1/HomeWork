// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */


   private CANSparkMax leftMotor1  = new CANSparkMax(0,CANSparkMax.MotorType.kBrushed);
   private CANSparkMax leftMotor2  = new CANSparkMax(1,CANSparkMax.MotorType.kBrushed);
   private CANSparkMax rightMotor1  = new CANSparkMax(2,CANSparkMax.MotorType.kBrushed);
   private CANSparkMax rightMotor2  = new CANSparkMax(3,CANSparkMax.MotorType.kBrushed);

   DifferentialDrive differentialDrive = new DifferentialDrive(leftMotor1,rightMotor1);

   private Joystick joy1 = new Joystick(0); //placeholder port
  @Override
  public void robotInit() {

    leftMotor2.follow(leftMotor1);
    rightMotor2.follow(rightMotor1);
    
    rightMotor1.setInverted(true);
    

  }

  @Override
  public void robotPeriodic() {

  }

  @Override
  public void autonomousInit() {

  }

  @Override
  public void autonomousPeriodic() {
  


    
  }

  @Override
  public void teleopInit() {

  }

  @Override
  public void teleopPeriodic() {
  //double speed = joy1.getRawAxis(1) * 0.6; //placeholder
  //double turn = joy1.getRawAxis(4) * 0.3; //placeholder
  //double left = speed + turn;
  //double right = speed -turn;


  differentialDrive.arcadeDrive(joy1.getRawAxis(1), joy1.getRawAxis(4));
  //leftMotor2.set(left);
  //rightMotor1.set(-right);
  //rightMotor2.set(-right);
  }

  @Override
  public void disabledInit() {

  }

  @Override
  public void disabledPeriodic() {

  }

  @Override
  public void testInit() {

  }

  @Override
  public void testPeriodic() {

  }

  @Override
  public void simulationInit() {

  }

  @Override
  public void simulationPeriodic() {

  }
}
