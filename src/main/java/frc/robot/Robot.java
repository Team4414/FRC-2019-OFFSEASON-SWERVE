package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.SwerveDriveModule.ModuleConfig;
import frc.robot.SwerveDriveModule.SwerveLocation;

public class Robot extends TimedRobot {

  public static final double kWheelBaseLength = 17;
  public static final double kWheelBaseWidth = 17;
  public static final double kSwerveDiagonal = Math.hypot(kWheelBaseLength, kWheelBaseWidth);
  private Joystick xbox;
  private static final double kJoystickDeadzone = 0.1;

  @Override
  public void robotInit() {
    xbox = new Joystick(0);
  }

  @Override
  public void robotPeriodic() {
    Drivetrain.getInstance().displayTurnAngles(); //push turn angles to SD
  }

  @Override
  public void autonomousInit() {
    Drivetrain.getInstance().enableAll(false); //disable all motors in auto for calibration
  }

  @Override
  public void autonomousPeriodic() {
    Drivetrain.getInstance().calibrateAll(); //calibrate all the potentiometers (hand-spin each module at least 1 rev to capture min and max sensor voltages)
  }

  @Override
  public void teleopInit() {
    Drivetrain.getInstance().zeroGyro(); //zero gyro for field relative control
    Drivetrain.getInstance().enableAll(true); //enable motors
  }

  @Override
  public void teleopPeriodic() {
    Drivetrain.getInstance().setFieldRelative(-deadZoneStick(0), deadZoneStick(1), -deadZoneStick(4)); //some sticks inverted cuz xbox
  }

  private double deadZoneStick(int stickId){
    if (Math.abs(xbox.getRawAxis(stickId)) < kJoystickDeadzone){
      return 0;
    }
    return xbox.getRawAxis(stickId);
  }
}
