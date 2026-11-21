package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrive extends SubsystemBase{
    
  public static final double kMaxSpeed = 3.0; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

  //Boolean for if swerve module informations should be published
  private static final boolean publishInfo = true;

  //Swerve info
  private final Translation2d m_frontLeftLocation;
  private final Translation2d m_frontRightLocation;
  private final Translation2d m_backLeftLocation;
  private final Translation2d m_backRightLocation;

  private final SwerveModule m_frontLeft;
  private final SwerveModule m_frontRight;
  private final SwerveModule m_backLeft;
  private final SwerveModule m_backRight;

  private final Pigeon2 m_gyro;

  private final SwerveDriveKinematics m_kinematics;

  private final SwerveDriveOdometry m_odometry;

  //Publishers for dashboard
  private DoublePublisher FLSpeedPub;
  private DoublePublisher FRSpeedPub;
  private DoublePublisher BLSpeedPub;
  private DoublePublisher BRSpeedPub;
  private DoublePublisher FLAnglePub;
  private DoublePublisher FRAnglePub;
  private DoublePublisher BLAnglePub;
  private DoublePublisher BRAnglePub;


  public SwerveDrive() {
    //Set up swerve
    m_frontLeftLocation = new Translation2d(7.375, 7.375);
    m_frontRightLocation = new Translation2d(7.375, -7.375);
    m_backLeftLocation = new Translation2d(-7.375, 7.375);
    m_backRightLocation = new Translation2d(-7.375, -7.375);

    m_frontLeft = new SwerveModule(26, 25, 10, 0.138672,true, false);
    m_frontRight = new SwerveModule(28, 27, 13, -0.306396, true, false);
    m_backLeft = new SwerveModule(24, 23, 11, 0.102051, true, false);
    m_backRight = new SwerveModule(22, 21, 12, 0.349365, true, false);

    m_gyro = new Pigeon2(30);

    m_kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

    m_odometry =
      new SwerveDriveOdometry(
          m_kinematics,
          m_gyro.getRotation2d(),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
          });

    m_gyro.reset();

    //Set up publishers
    if (publishInfo){
      NetworkTableInstance inst = NetworkTableInstance.getDefault();
      NetworkTable table = inst.getTable("FMSInfo");
      FLSpeedPub = table.getDoubleTopic("FLSpeed").publish();
      FRSpeedPub = table.getDoubleTopic("FRSpeed").publish();
      BLSpeedPub = table.getDoubleTopic("BLSpeed").publish();
      BRSpeedPub = table.getDoubleTopic("BRSpeed").publish();
      FLAnglePub = table.getDoubleTopic("FLAngle").publish();
      FRAnglePub = table.getDoubleTopic("FRAngle").publish();
      BLAnglePub = table.getDoubleTopic("BLAngle").publish();
      BRAnglePub = table.getDoubleTopic("BRAngle").publish();
    }
  }
  
  //Update values of publishers
  public void updatePub(){
    FLSpeedPub.set(m_frontLeft.getState().speedMetersPerSecond);
    FRSpeedPub.set(m_frontRight.getState().speedMetersPerSecond);
    BLSpeedPub.set(m_backLeft.getState().speedMetersPerSecond);
    BRSpeedPub.set(m_backRight.getState().speedMetersPerSecond);
    FLAnglePub.set(m_frontLeft.getState().angle.getDegrees());
    FRAnglePub.set(m_frontRight.getState().angle.getDegrees());
    BLAnglePub.set(m_backLeft.getState().angle.getDegrees());
    BRAnglePub.set(m_backRight.getState().angle.getDegrees());
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(
      double xSpeed, double ySpeed, double rot, boolean fieldRelative, double periodSeconds) {
    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            ChassisSpeeds.discretize(
                fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                    : new ChassisSpeeds(xSpeed, ySpeed, rot),
                periodSeconds));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]); //Frontleft
    m_frontRight.setDesiredState(swerveModuleStates[1]); //Frontright
    m_backLeft.setDesiredState(swerveModuleStates[2]); //Backleft
    m_backRight.setDesiredState(swerveModuleStates[3]); //Backright
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
        });
  }

  @Override
  public void periodic(){
    updateOdometry();
    updatePub();
  }

}
