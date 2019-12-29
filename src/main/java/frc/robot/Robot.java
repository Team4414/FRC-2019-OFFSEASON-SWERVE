/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import frc.robot.subsystems.Drivetrain;

public class Robot extends TimedRobot {
  private final Joystick m_controller = new Joystick(0);

  @Override
  public void autonomousPeriodic() {
    driveWithJoystick();
    Drivetrain.getInstance().updateOdometry();
  }

  @Override
  public void teleopPeriodic() {
    driveWithJoystick();
  }

  Rotation2d m_lastRot;
  private void driveWithJoystick() {
    Translation2d speed = new Translation2d( m_controller.getRawAxis(2),m_controller.getRawAxis(2));
    if (m_controller.getMagnitude() < .07) { speed = new Translation2d();}
    speed = speed.times(Math.pow(speed.getNorm(),2));
    Rotation2d rot = new Rotation2d(m_controller.getDirectionRadians());
    if (m_controller.getMagnitude() < .5) { rot = m_lastRot;}
    m_lastRot = rot;
    Drivetrain.getInstance().setFieldRelativeWithHeading(speed, rot);
  }
}
