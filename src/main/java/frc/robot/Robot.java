package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Limelight.CAM_MODE;
import frc.robot.Limelight.LED_STATE;
import frc.robot.Limelight.Side;
import frc.robot.SwerveDriveModule.ModuleConfig;
import frc.robot.SwerveDriveModule.SwerveLocation;

public class Robot extends TimedRobot {

  public static final double kWheelBaseLength = 17;
  public static final double kWheelBaseWidth = 17;
  public static final double kSwerveDiagonal = Math.hypot(kWheelBaseLength, kWheelBaseWidth);
  public static Joystick driveStick;
  public static Joystick xbox;
  public static Joystick turnStick;
  private static final double kJoystickDeadzone = 0.06;
  private static final double kRotationScalar = 1.4;
  private static final double kRotationMax = 0.8;
  private static final double kTranslationScalar = 1.4;

  private static VictorSPX mPP;

  public static boolean mSticksAreInDeadzone = false;

  public static PowerDistributionPanel pdp = new PowerDistributionPanel(0);
  Limelight mLimelight;

  @Override
  public void robotInit() {
    driveStick = new Joystick(0);
    turnStick = new Joystick(1);
    xbox = new Joystick(2);

    mPP = new VictorSPX(11);
    mLimelight = new Limelight(Side.BALL);
  }

  @Override
  public void robotPeriodic() {
    // Drivetrain.getInstance().displayTurnAngles(); //push turn angles to SD
    // System.out.println(stickToDegee(turnStick, 0, 1));
    // System.out.println(Drivetrain.getInstance().mFLmodule.getError());
    // System.out.println( deadZoneStick(driveStick, 1, 0));
    // SmartDashboard.putNumber("Swerve Error", Drivetrain.getInstance().mFLmodule.getError());
    // System.out.println(Math.hypot(deadZoneStick(driveStick, 0, 1), deadZoneStick(driveStick, 1, 0)));
    Drivetrain.getInstance().mFLmodule.updateHeadingLoop();
    Drivetrain.getInstance().mFRmodule.updateHeadingLoop();
    Drivetrain.getInstance().mBLmodule.updateHeadingLoop();
    Drivetrain.getInstance().mBRmodule.updateHeadingLoop();

    // System.out.println((Drivetrain.getInstance().mFLmodule.mError - Drivetrain.getInstance().mFLmodule.mLastError) / 0.02d);
  }

  @Override
  public void disabledInit() {
    mLimelight.setCamMode(CAM_MODE.VISION);
    mLimelight.setLED(LED_STATE.ON);
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
    
    // Drivetrain.getInstance().mFLmodule.updateHeadingLoop();
    
    // System.out.println(Drivetrain.getInstance().mFLmodule.mError + "\t" + Drivetrain.getInstance().mFLmodule.mLastError);

    mSticksAreInDeadzone = (getDeadZoneStick(driveStick, 0, 1) && getDeadZoneStick(driveStick, 1, 0) && getDeadZoneStick(turnStick, 0, 1) && getDeadZoneStick(driveStick, 1, 0));

    // System.out.println(getDeadZoneStick(driveStick, 0, 1) && getDeadZoneStick(driveStick, 1, 0));
    // Drivetrain.getInstance().setFieldRelative(0, 0, stickToDegee(turnStick , 0, 1), true);

    // if (driveStick.getRawButton(12)){
    //   Drivetrain.getInstance().zeroGyro();
    // }

    if (!driveStick.getRawButton(11)){
       Drivetrain.getInstance().setFieldRelative(getScalar(driveStick, 0, 1, kTranslationScalar) * deadZoneStick(driveStick, 0, 1), -getScalar(driveStick, 0, 1, kTranslationScalar) * deadZoneStick(driveStick, 1, 0), stickToDegee(turnStick , 0, 1), true);
    }else if(driveStick.getRawButton(11)){
      lastHeading = Drivetrain.getInstance().getAngle();
        Drivetrain.getInstance().setFieldRelative(getScalar(driveStick, 0, 1, kTranslationScalar) * deadZoneStick(driveStick, 0, 1), -getScalar(driveStick, 0, 1, kTranslationScalar) * deadZoneStick(driveStick, 1, 0), -deadZoneStick(turnStick, 0, 1), false);
    }else if (driveStick.getRawButton(12)){
      Drivetrain.getInstance().zeroGyro();
      // mLimelight.setLED(LED_STATE.ON);
      // mLimelight.setCamMode(CAM_MODE.VISION);
      // Drivetrain.getInstance().setFieldRelative(0.1 * mLimelight.tX(), -getScalar(driveStick, 0, 1, kTranslationScalar) * deadZoneStick(driveStick, 1, 0), stickToDegee(turnStick , 0, 1), false);
    }else{
      mLimelight.setLED(LED_STATE.OFF);
    }
    if(turnStick.getRawButton(12)){
      mPP.set(ControlMode.PercentOutput, -1);
    }else if(turnStick.getRawButton(11)){
      mPP.set(ControlMode.PercentOutput, 1);
    }else{
      mPP.set(ControlMode.PercentOutput, -0.166);
    }
  }

  private boolean getDeadZoneStick(Joystick stick, int stickId, int otherStick){
    // if (Math.abs(stick.getRawAxis(stickId)) < kJoystickDeadzone){
    //   // mSticksAreInDeadzone = true;
    //   return true;
    // }
    // // mSticksAreInDeadzone = false;
    // return false;
    // System.out.println(Math.hypot(stick.getRawAxis(stickId), stick.getRawAxis(otherStick)));

    if (Math.hypot(stick.getRawAxis(stickId), stick.getRawAxis(otherStick)) < kJoystickDeadzone){
      return true;
    }else{
      return false;
    }
  }

  private double deadZoneStick(Joystick stick, int stickId, int otherStick){
    double scaler = 1;
    
    if (getDeadZoneStick(stick, stickId, otherStick)){
      return 0;
    }

    //scale for square sticks
    // if (Math.hypot(stick.getRawAxis(stickId), stick.getRawAxis(otherStick)) > 1){
    //   return stick.getRawAxis(stickId) / (Math.hypot(stick.getRawAxis(stickId), stick.getRawAxis(otherStick)));
    // }

    // if (Math.hypot(stick.getRawAxis(stickId), stick.getRawAxis(otherStick)) > 1){
    //   scaler = Math.hypot(stick.getRawAxis(stickId), stick.getRawAxis(otherStick));
    // }
    return stick.getRawAxis(stickId);
  }
  
  //for translate
  private double getScalar(Joystick stick, int stickX, int stickY, double scalar){
    // return (Math.pow(Math.abs(deadZoneStick(stick, stickId)), scalar) * Math.signum(deadZoneStick(stick, stickId)));
    return Math.pow(Math.hypot(deadZoneStick(stick, stickX, stickY), deadZoneStick(stick, stickY, stickX)), kTranslationScalar);
    // return 1;
    // return Math.pow((Math.hypot(deadZoneStick(stick, stickX, stickY), deadZoneStick(stick, stickY, stickX))), 2);
  }

  //inverse tan (-x / y)
  double lastHeading = 0;
  // double startTime;
  //atan lag is 0.0005
  private double stickToDegee(Joystick stick, int stickX, int stickY){
    // startTime = Timer.getFPGATimestamp();\

    
    double hyp = Math.hypot(stick.getRawAxis(stickX), -stick.getRawAxis(stickY));
    if (hyp > 0.5){
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
