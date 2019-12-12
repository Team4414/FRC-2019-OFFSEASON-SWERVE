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
  private static final double kTranslationScalar = 1; //1.4

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
    Drivetrain.getInstance().zeroGyro();
    Drivetrain.getInstance().zeroPosition();
  }

  @Override
  public void robotPeriodic() {
    Drivetrain.getInstance().mFLmodule.updateHeadingLoop();
    Drivetrain.getInstance().mFRmodule.updateHeadingLoop();
    Drivetrain.getInstance().mBLmodule.updateHeadingLoop();
    Drivetrain.getInstance().mBRmodule.updateHeadingLoop();

    Drivetrain.getInstance().displayTurnAnglesRaw();

    Drivetrain.getInstance().updatePosition();

    System.out.println("X: " + Drivetrain.getInstance().getPosition().x + "\t Y: " + Drivetrain.getInstance().getPosition().y + "\t" + Drivetrain.getInstance().getAngle());
  }

  @Override
  public void disabledInit() {
    mLimelight.setCamMode(CAM_MODE.VISION);
    mLimelight.setLED(LED_STATE.ON);
  }

  @Override
  public void autonomousInit() {
    Drivetrain.getInstance().enableAll(false); //disable all motors in auto for calibration
  }

  @Override
  public void autonomousPeriodic() {
    Drivetrain.getInstance().calibrateAll(); //calibrate all the potentiometers (hand-spin each module at least 1 rev to capture min and max sensor voltages)
    // Drivetrain.getInstance().setFieldRelative(-deadZoneStick(0), deadZoneStick(1), xbox.getPOV(), true);
  }

  @Override
  public void teleopInit() {
    Drivetrain.getInstance().zeroGyro(); //zero gyro for field relative control
    Drivetrain.getInstance().enableAll(true); //enable motors
    Drivetrain.getInstance().zeroPosition();
  }

  @Override
  public void teleopPeriodic() {

    mSticksAreInDeadzone = (getDeadZoneStick(driveStick, 0, 1) && getDeadZoneStick(driveStick, 1, 0) && getDeadZoneStick(turnStick, 0, 1) && getDeadZoneStick(driveStick, 1, 0));

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

    if (Math.hypot(stick.getRawAxis(stickId), stick.getRawAxis(otherStick)) < kJoystickDeadzone){
      return true;
    }else{
      return false;
    }
  }

  private double deadZoneStick(Joystick stick, int stickId, int otherStick){

    if (getDeadZoneStick(stick, stickId, otherStick)){
      return 0;
    }

    return stick.getRawAxis(stickId);
  }
  
  //for translate
  private double getScalar(Joystick stick, int stickX, int stickY, double scalar){
    return Math.pow(Math.hypot(deadZoneStick(stick, stickX, stickY), deadZoneStick(stick, stickY, stickX)), kTranslationScalar);
  }

  double lastHeading = 0;
  private double stickToDegee(Joystick stick, int stickX, int stickY){
    double hyp = Math.hypot(stick.getRawAxis(stickX), -stick.getRawAxis(stickY));
    if (hyp > 0.5){
      lastHeading = Math.toDegrees(Math.atan2(stick.getRawAxis(stickX) / hyp, -stick.getRawAxis(stickY) / hyp)); //negate stick y and x for invert stick
    }
    return lastHeading;
  }
}
