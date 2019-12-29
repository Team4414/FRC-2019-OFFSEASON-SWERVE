/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.controller.PIDController;

/**
 * Represents a swerve drive style drivetrain.
 */
public class Drivetrain extends SubsystemBase {

  private static Drivetrain instance;
  public static Drivetrain getInstance(){
      if (instance == null)
          instance = new Drivetrain();
      return instance;
  }


  public static final double kMaxSpeed = 3.0; // 3 meters per second
  public static final double kMaxAngularSpeed = kMaxSpeed * 0.381; // Theoretical Max Rotation

  private final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
  private final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
  private final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

  private final SwerveModule m_frontLeft = new SwerveModule(1, 2);
  private final SwerveModule m_frontRight = new SwerveModule(3, 4);
  private final SwerveModule m_backLeft = new SwerveModule(5, 6);
  private final SwerveModule m_backRight = new SwerveModule(7, 8);

  private final AnalogGyro m_gyro = new AnalogGyro(0);

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
  );

  private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, getAngle());

  private final PIDController m_headingController = new PIDController(1, 0, 0);

  public Drivetrain() {
    m_headingController.enableContinuousInput(-Math.PI, Math.PI);
    m_gyro.reset();
  }

  @Override
  public void periodic() {
    
  }

  public Rotation2d getAngle() {
    // Negating the angle because WPILib gyros are CW positive.
    return Rotation2d.fromDegrees(-m_gyro.getAngle());
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(Translation2d speed, double rot, boolean fieldRelative) {

    var swerveModuleStates = m_kinematics.toSwerveModuleStates(
        fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
            speed.times(kMaxSpeed).getX(), speed.times(kMaxSpeed).getY(), rot * kMaxAngularSpeed, getAngle())
            : new ChassisSpeeds(speed.times(kMaxSpeed).getX(), speed.times(kMaxSpeed).getY(), rot)
    );
    SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  public void setRobotRelative(Translation2d speed, double rot) {
    drive(speed, rot  , false);
  }
  public void setFieldRelative(Translation2d speed, double rot) {
    drive(speed, rot , true);
  }
  public void setFieldRelativeWithHeading(Translation2d speed, Rotation2d theta) {
    setFieldRelative(speed, m_headingController.calculate(getAngle().getRadians(), theta.getRadians()) );
  }


  public void updateOdometry() {
    m_odometry.update(
        getAngle(),
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_backLeft.getState(),
        m_backRight.getState()
    );
  }
}
