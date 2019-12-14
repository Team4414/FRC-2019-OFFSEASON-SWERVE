package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveDriveModule{

    public static enum SwerveLocation{
        FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT;
    }

    private AnalogInput mTurnPot;
    private IMotorController mTurnMotor;
    private TalonSRX mDriveMotor;

    private ModuleConfig mConfig;
    private SwerveLocation mLocation;

    private final double kP = (0.007); //0.012
    private final double kD = (0.00024);  //0.00024
    private final double kTimestep = 0.02d;

    public static final double kReversableWheelThreshold = 5;

    // PIDCommand

    public double mTurnSetpoint;
    public double mVelSetpoint;
    public double mLastError;
    public double mLastAngle = 0;
    public double mError = 0;
    public double mInverter = 0;
    public double mTurnSetpointDesired = 0;

    public double mLastPosition = 0;

    private boolean mTurnPhase = false;

    public boolean motorEnabled = true;
    
    public static final double kFeet2Ticks = 6579;
    public static final double kTicks2Feet = 1/kFeet2Ticks;
    public static final double kFPS2NativeU = kFeet2Ticks / 10;
    public static final double kNativeU2FPS = 1 / kFPS2NativeU;

    public static final double kMaxTurnPower = 0.83;
    public double kDriveF = 0;

    public SwerveDriveModule(SwerveLocation location, ModuleConfig config, IMotorController turnMotor, int kDrivePort, int kTurnSensor, boolean turnPhase){
        mTurnMotor = turnMotor;
        mDriveMotor = new TalonSRX(kDrivePort);

        mDriveMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        mDriveMotor.config_kP(0, 0.060); //0.1
        mDriveMotor.config_kF(0, 0.083);
        mDriveMotor.setSensorPhase(true);
        mDriveMotor.overrideLimitSwitchesEnable(false);
        mDriveMotor.overrideSoftLimitsEnable(false);
        mDriveMotor.configContinuousCurrentLimit(40);

        mDriveMotor.configOpenloopRamp(0.08);
        mDriveMotor.configVoltageCompSaturation(12);
        mDriveMotor.enableVoltageCompensation(true);

        mTurnSetpoint = 0;
        mLastPosition = getDistance();

        mTurnPhase = turnPhase;

        mLocation = location;
        mConfig = config;
        mTurnPot = new AnalogInput(kTurnSensor);
    }

    public void config(ModuleConfig config){
        mConfig = config;
    }

    public ModuleConfig getConfig(){
        return mConfig;
    }

    public double getSpeed(){
        return mDriveMotor.getSelectedSensorVelocity() * kNativeU2FPS;
    }

    /**
     * Function for calibrating potentiometers.
     */
    public void calibrate(){

        SmartDashboard.putNumber(getLocationAsString() + " Swerve Pot Max", mConfig.turnMaxVoltage);
        SmartDashboard.putNumber(getLocationAsString() + " Swerve Pot Min", mConfig.turnMinVoltage);

        mConfig.calibrateTurnPot(mTurnPot);
    }

    double maxCurrent = 0;

    public double getCurrent(){
        if (Robot.pdp.getCurrent(2) > maxCurrent)
            maxCurrent = Robot.pdp.getCurrent(2);
        return maxCurrent;
    }

    @Deprecated
    public void zero(){
        //Should probably remove this no need to have.
        SmartDashboard.putNumber(getLocationAsString() + " Swerve Zero", mConfig.zero);
        mConfig.zero(mTurnPot);
    }

    public double getError(){
        return boundHalfDegrees((-mTurnSetpoint - getAngle()));
    }

    public double getAngle(){
        return boundHalfDegrees(mConfig.getAngle(mTurnPot));
    }

    public double getRawAngle(){
        return mConfig.getRawAngle(mTurnPot);
    }

    @Deprecated
    public double setRawSpeed(double speed){
        mDriveMotor.set(ControlMode.PercentOutput, speed);
        return mDriveMotor.getSelectedSensorVelocity() * kNativeU2FPS;
    }

    public Position mModulePositionDelta;
    public double dist = 0;

    public void updateOdom(){
        dist = getDistance() - mLastPosition;
        mLastPosition = getDistance();

        mModulePositionDelta = new Position((Math.cos(Math.toRadians(getRobotRelativeHeading())) * dist),
                            (Math.sin(Math.toRadians(getRobotRelativeHeading())) * dist));
    }

    public Position getXYDelta(){
        return mModulePositionDelta;
    } 
    
    public double getDistance(){
        return -mDriveMotor.getSelectedSensorPosition() * kTicks2Feet;
    }

    public double getRobotRelativeHeading(){
        return Drivetrain.getInstance().getAngle() - (-getAngle());
    }

    public String getLocationAsString(){
        switch (mLocation){
            case FRONT_LEFT:
                return "Front Left";
            case FRONT_RIGHT:
                return "Front Right";
            case BACK_LEFT:
                return "Back Left";
            case BACK_RIGHT:
                return "Back Right";
        }
        return "UNKNOWN";
    }

    public static double boundHalfDegrees(double angle_degrees) {
        while (angle_degrees >= 180.0) angle_degrees -= 360.0;
        while (angle_degrees < -180.0) angle_degrees += 360.0;
        return angle_degrees;
    }

    public double getModifiedError(){
        return (boundHalfDegrees(getError()))/180;
    }

    public void setSteeringDegrees(double deg){
        mTurnSetpointDesired = deg;
    }

    public void updateHeadingLoop(){

        if (!Robot.mSticksAreInDeadzone){
            mTurnSetpoint = mTurnSetpointDesired;
        }

        mError = getError();
        mInverter = (mTurnPhase) ? -1 : 1;

        if (motorEnabled){
            mTurnMotor.set(ControlMode.PercentOutput, mInverter * ((mError * kP) + (((mError - mLastError) / kTimestep) * kD)));
            mDriveMotor.set(ControlMode.Velocity, Math.cos(Math.toRadians(mError)) * mVelSetpoint);
        }

        mLastError = mError;
    }
    public void set(double degrees, double power, boolean flip){

        if (!flip){
            double supplement = degrees > 0 ? degrees - 180 : 180 + degrees;

            if(Math.abs(supplement-mLastAngle) <= 90){
                setSteeringDegrees(supplement);
                setDrivePower(-power);
                mLastAngle = supplement;
            }
            else {
                setSteeringDegrees(degrees);
                setDrivePower(power);
                mLastAngle = degrees;
            }
        }else{
            setSteeringDegrees(degrees);
            setDrivePower(power);
        }
    }

    public void setDrivePower(double fps){
        mVelSetpoint = fps * kFPS2NativeU;
    }

    public static class ModuleConfig{
        public double turnMinVoltage = 13;
        public double turnMaxVoltage = 0;
        public double zero = 0;

        public ModuleConfig(double turnMinVoltage, double turnMaxVoltage, double zero){
            this.turnMinVoltage = turnMinVoltage;
            this.turnMaxVoltage = turnMaxVoltage;
            this.zero = zero;
        }

        public ModuleConfig(){};

        public void calibrateTurnPot(AnalogInput pot){
            if (pot.getVoltage() < turnMinVoltage){
                turnMinVoltage = pot.getVoltage();
            }
    
            if (pot.getVoltage() > turnMaxVoltage){
                turnMaxVoltage = pot.getVoltage();
            }
        }

        @Deprecated
        public void zero(AnalogInput pot){
            zero = getRawAngle(pot);
        } 

        public double getAngle(AnalogInput pot){
           return (getRawAngle(pot) - zero) % 360;
        }

        public double getRawAngle(AnalogInput pot){
            return (pot.getVoltage() - turnMinVoltage) * (360d / (turnMaxVoltage - turnMinVoltage));
        }
    }
}