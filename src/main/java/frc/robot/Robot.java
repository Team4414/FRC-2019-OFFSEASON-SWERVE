package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.SwerveDriveModule.ModuleConfig;
import frc.robot.SwerveDriveModule.SwerveLocation;

public class Robot extends TimedRobot {

  public static final double kWheelBaseLength = 17;
  public static final double kWheelBaseWidth = 17;
  public static final double kSwerveDiagonal = Math.hypot(kWheelBaseLength, kWheelBaseWidth);
  public static Joystick driveStick;
  public static Joystick xbox;
  public static Joystick turnStick;
  private static final double kJoystickDeadzone = 0.03;
  private static final double kRotationScalar = 1.4;
  private static final double kRotationMax = 0.8;
  private static final double kTranslationScalar = 1.8;

  public static PowerDistributionPanel pdp = new PowerDistributionPanel(0);

  @Override
  public void robotInit() {
    driveStick = new Joystick(0);
    turnStick = new Joystick(1);
    xbox = new Joystick(2);
  }

  @Override
  public void robotPeriodic() {
    // Drivetrain.getInstance().displayTurnAngles(); //push turn angles to SD
    // System.out.println(stickToDegee(turnStick, 0, 1));
    // System.out.println(Drivetrain.getInstance().mFLmodule.getCurrent());
  }

  @Override
  public void autonomousInit() {
    Drivetrain.getInstance().enableAll(true); //disable all motors in auto for calibration
  }

  @Override
  public void autonomousPeriodic() {
    // Drivetrain.getInstance().calibrateAll(); //calibrate all the potentiometers (hand-spin each module at least 1 rev to capture min and max sensor voltages)
    // Drivetrain.getInstance().setFieldRelative(-deadZoneStick(0), deadZoneStick(1), xbox.getPOV(), true);
  }

  @Override
  public void teleopInit() {
    Drivetrain.getInstance().zeroGyro(); //zero gyro for field relative control
    Drivetrain.getInstance().enableAll(true); //enable motors
  }

  @Override
  public void teleopPeriodic() {

    if (!turnStick.getRawButton(11)){
      Drivetrain.getInstance().setFieldRelative(getScalar(driveStick, 0, 1) * deadZoneStick(driveStick, 0, 1), -getScalar(driveStick, 0, 1) * deadZoneStick(driveStick, 1, 0), stickToDegee(turnStick , 0, 1), true);
    }else{
      lastHeading = Drivetrain.getInstance().getAngle();
      Drivetrain.getInstance().setFieldRelative(getScalar(driveStick, 0, 1) * deadZoneStick(driveStick, 0, 1), -getScalar(driveStick, 0, 1) * deadZoneStick(driveStick, 1, 0), -deadZoneStick(turnStick, 0, 1), false);
      // Drivetrain.getInstance().setFieldRelative(getScalar(driveStick, 0, 1) * deadZoneStick(driveStick, 0), getScalar(driveStick, 0, 1) * deadZoneStick(driveStick, 1), stickToDegee(turnStick, 0, 1), true);
      // Drivetrain.getInstance().setFieldRelative(getScaledStick(driveStick, 0, kTranslationScalar), -getScaledStick(driveStick, 1, kTranslationScalar), -getScaledStick(turnStick ,0, kRotationScalar) * kRotationMax, false);
    }
    if (driveStick.getRawButton(12)){
      Drivetrain.getInstance().zeroGyro();
    }
  }

  private double deadZoneStick(Joystick stick, int stickId, int otherStick){
    double scaler = 1;
    if (Math.abs(stick.getRawAxis(stickId)) < kJoystickDeadzone){
      return 0;
    }
    // if (Math.hypot(stick.getRawAxis(stickId), stick.getRawAxis(otherStick)) > 1){
    //   scaler = Math.hypot(stick.getRawAxis(stickId), stick.getRawAxis(otherStick));
    // }
    return stick.getRawAxis(stickId) / scaler;
  }
  
  //for translate
  private double getScalar(Joystick stick, int stickX, int stickY){
    // return (Math.pow(Math.abs(deadZoneStick(stick, stickId)), scalar) * Math.signum(deadZoneStick(stick, stickId)));
    // return Math.pow(Math.hypot(deadZoneStick(stick, stickX, stickY), deadZoneStick(stick, stickY, stickX)), kTranslationScalar);
    return 1;
  }

  //inverse tan (-x / y)
  double lastHeading = 0;
  // double startTime;
  //atan lag is 0.0005
  private double stickToDegee(Joystick stick, int stickX, int stickY){
    // startTime = Timer.getFPGATimestamp();
    double hyp = Math.hypot(stick.getRawAxis(stickX), -stick.getRawAxis(stickY));
    if (hyp > 0.1){
      lastHeading = Math.toDegrees(Math.atan2(stick.getRawAxis(stickX) / hyp, -stick.getRawAxis(stickY) / hyp)); //negate stick y and x for invert stick
    }
    // System.out.println("Atan lag: " + (Timer.getFPGATimestamp() - startTime));
    return lastHeading;
  }

  @Override
  public void testPeriodic() {
    // Drivetrain.getInstance().enableAll(false);
  }
}
