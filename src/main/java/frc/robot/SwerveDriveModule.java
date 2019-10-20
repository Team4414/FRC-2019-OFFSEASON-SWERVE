package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Notifier;
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

    private Notifier turnPID;
    private final double kP = (0.015);
    private final double kD = (0.75); 
    private final double kTimestep = 0.02;

    private double mTurnSetpoint;
    private double mVelSetpoint;
    private double mLastError;
    private double mLastAngle = 0;

    public static boolean motorEnabled = true;
    
    public static final double kFeet2Ticks = 6579;
    public static final double kTicks2Feet = 1/kFeet2Ticks;
    public static final double kFPS2NativeU = kFeet2Ticks / 10;
    public static final double kNativeU2FPS = 1 / kFPS2NativeU;

    public SwerveDriveModule(SwerveLocation location, ModuleConfig config, IMotorController turnMotor, int kDrivePort, int kTurnSensor, boolean turnPhase){
        mTurnMotor = turnMotor;
        mDriveMotor = new TalonSRX(kDrivePort);

        mDriveMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        mDriveMotor.config_kP(0, 0.20);
        mDriveMotor.setSensorPhase(true);

        mTurnSetpoint = 0;

        mLocation = location;
        mConfig = config;
        mTurnPot = new AnalogInput(kTurnSensor);

        turnPID = new Notifier(() -> {

            double error = getError();
            double inverter = (turnPhase) ? -1 : 1;

            if (motorEnabled){
                mTurnMotor.set(ControlMode.PercentOutput, (error * kP * inverter) + ((getError() - mLastError) * kD * inverter * kTimestep));
                mDriveMotor.set(ControlMode.Velocity, mVelSetpoint);
            }else{
                mTurnMotor.set(ControlMode.PercentOutput, 0);
                mDriveMotor.set(ControlMode.PercentOutput, 0);
            }
            mLastError = error;
        });

        turnPID.startPeriodic(kTimestep);
    }

    public void config(ModuleConfig config){
        mConfig = config;
    }

    public ModuleConfig getConfig(){
        return mConfig;
    }

    /**
     * Function for calibrating potentiometers.
     */
    public void calibrate(){

        SmartDashboard.putNumber(getLocationAsString() + " Swerve Pot Max", mConfig.turnMaxVoltage);
        SmartDashboard.putNumber(getLocationAsString() + " Swerve Pot Min", mConfig.turnMinVoltage);

        mConfig.calibrateTurnPot(mTurnPot);
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
        mTurnSetpoint = deg;
    }

    public void set(double degrees, double power){
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

        // setSteeringDegrees(degrees);
        // mVelSetpoint = power * 4096;
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