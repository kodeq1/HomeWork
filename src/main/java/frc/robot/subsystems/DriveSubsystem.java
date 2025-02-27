
package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OIConstants;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class DriveSubsystem extends SubsystemBase {


  PS4Controller PS4Controller = new PS4Controller(OIConstants.kDriverControllerPort);

  

  private final SparkMax leftLeader = new SparkMax(1, MotorType.kBrushed);
  private final SparkMax leftFollower = new SparkMax(2, MotorType.kBrushed);

  private final SparkMax rightLeader = new SparkMax(3, MotorType.kBrushed);
  private final SparkMax rightFollower = new SparkMax(4, MotorType.kBrushed);

  private final DifferentialDrive drive = new DifferentialDrive(leftLeader, rightLeader);

  private final Encoder leftEncoder = 
        new Encoder(
          DriveConstants.kLeftEncoderPorts[0],
          DriveConstants.kLeftEncoderPorts[1],
          DriveConstants.kLeftEncoderReversed
        );

  private final Encoder rightEncoder =
        new Encoder(
          DriveConstants.kRightEncoderPorts[0],
          DriveConstants.kRightEncoderPorts[1],
          DriveConstants.kRightEncoderReversed);

  private final ADXRS450_Gyro gyro = new ADXRS450_Gyro();
        
  double outputYcoeffector = -1;

    //Odometry/kinematics definition(robot position/angle/velocity)
    private final DifferentialDriveOdometry odometry;
    private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(DriveConstants.kTrackwidthMeters);
    private final PIDController xPidController = new PIDController(AutoConstants.kPXController, AutoConstants.kIXController, AutoConstants.kDXController);
    private final PIDController yawPidController = new PIDController(AutoConstants.kPYawController, AutoConstants.KIYawController, AutoConstants.kDYawController);

    public Pose2d pose;
    public DifferentialDrivetrainSim drivetrainSimulator;
    public EncoderSim leftEncoderSim;
    public EncoderSim rightEncoderSim;
    private final Field2d fieldSim;
    private final ADXRS450_GyroSim gyroSim;

    double cuurentAngle = 0;

  public DriveSubsystem() {


    leftLeader.setCANTimeout(250);
    rightLeader.setCANTimeout(250);
    leftFollower.setCANTimeout(250);
    rightFollower.setCANTimeout(250);


    SparkMaxConfig config = new SparkMaxConfig();
    config.voltageCompensation(12);
    config.smartCurrentLimit(60);

    //left motors config
    config.follow(leftLeader);
    leftFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    config.follow(rightLeader);
    rightFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    //right motors config
    config.disableFollowerMode();
    rightLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    config.inverted(true);
    leftLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //encoder config
    leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);

    resetEncoders();

    odometry = 
    new DifferentialDriveOdometry(
        Rotation2d.fromDegrees(getHeading()),
                leftEncoder.getDistance(),
                rightEncoder.getDistance());

      if (RobotBase.isSimulation()) {
        drivetrainSimulator = new DifferentialDrivetrainSim(
          DriveConstants.kDrivetrainPlant, 
          DriveConstants.kDriveGearbox, 
          DriveConstants.kDriveGearing, 
          DriveConstants.kTrackwidthMeters, 
          DriveConstants.kWheelDiameterMeters / 2.0, 
          VecBuilder.fill(0, 0, 0.0001, 0.1, 0.1, 0.005, 0.005));
          
          leftEncoderSim = new EncoderSim(leftEncoder);
          rightEncoderSim = new EncoderSim(rightEncoder);
          gyroSim = new ADXRS450_GyroSim(gyro);
          fieldSim = new Field2d();
          SmartDashboard.putData("Field", fieldSim);
            
      } else {
          leftEncoderSim = null;
          rightEncoderSim = null;
          gyroSim = null;
          fieldSim = null;
      }
        
   }
  
                  
    @Override
  public void periodic() {
        //Updates odometry
        pose = odometry.update(
            Rotation2d.fromDegrees(getHeading()), 
            leftEncoder.getDistance(), 
            rightEncoder.getDistance());

        //Updates robots pose
        fieldSim.setRobotPose(pose);

  }

  public void simulationPeriodic() {
    //sets simulaton motors input to real motors
    drivetrainSimulator.setInputs(
        leftLeader.get() * RobotController.getBatteryVoltage(), 
        rightLeader.get() * RobotController.getBatteryVoltage());
    drivetrainSimulator.update(0.02);

    //sets sim encoders distance and velocity in meters
    leftEncoderSim.setDistance(drivetrainSimulator.getLeftPositionMeters());
    leftEncoderSim.setRate(drivetrainSimulator.getLeftVelocityMetersPerSecond());

    rightEncoderSim.setDistance(drivetrainSimulator.getRightPositionMeters());
    rightEncoderSim.setRate(drivetrainSimulator.getRightVelocityMetersPerSecond());

    //sets simulation gyros angle 
    //(-) because .getheading() is counterclockwise positive and setangle is clockwise positive
    gyroSim.setAngle(-drivetrainSimulator.getHeading().getDegrees());
    
  }



  SlewRateLimiter outputYLimiter = new SlewRateLimiter(0.5);
  double outputY;
  double outputX;

  public void arcadeDrive(double forward, double rotation) {
    outputY = forward*0.8;
      outputX = rotation*0.55;
    if(PS4Controller.getR1Button()){
      outputY = outputYLimiter.calculate(outputYcoeffector*forward*0.9);
      outputX = rotation*0.55;
    }
      
    drive.arcadeDrive(outputY, outputX);
  }


  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftLeader.setVoltage(leftVolts);
    rightLeader.setVoltage(rightVolts);
    drive.feed();
  }


  public void drive(ChassisSpeeds speeds){
    drive.feed();  

    xPidController.setSetpoint(speeds.vxMetersPerSecond);
    yawPidController.setSetpoint(speeds.omegaRadiansPerSecond);

    drive.arcadeDrive(xPidController.calculate(getChassisSpeeds().vxMetersPerSecond), yawPidController.calculate(getChassisSpeeds().omegaRadiansPerSecond));
  }

  public double getAverageEncoderDistance() {
    return (leftEncoder.getDistance() + rightEncoder.getDistance()) / 2.0;
  }

  public void setMaxOutput(double maxOutput) {
    drive.setMaxOutput(maxOutput);
  }

  //sets robots heading to zero
  public void zeroHeading() {
    gyro.reset();
  }


  //reset functions
      
  private void resetEncoders() {
    leftEncoder.reset();
    rightEncoder.reset();

  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    drivetrainSimulator.setPose(pose);
    odometry.resetPosition(
        Rotation2d.fromDegrees(pose.getRotation().getDegrees()),
        leftEncoder.getDistance(),
        rightEncoder.getDistance(),
        pose);
  }


  //getters

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
}

  public ChassisSpeeds getChassisSpeeds(){
    return m_kinematics.toChassisSpeeds(getWheelSpeeds());
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
}

  private double getHeading() {
    return Math.IEEEremainder(gyro.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
}

public Encoder getLeftEncoder() {
  return leftEncoder;
}

public Encoder getRightEncoder() {
  return rightEncoder;
}

}

