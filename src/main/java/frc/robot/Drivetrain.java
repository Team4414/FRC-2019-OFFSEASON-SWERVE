package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.CalibrationMode;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.SwerveDriveModule.ModuleConfig;
import frc.robot.SwerveDriveModule.SwerveLocation;

public class Drivetrain extends Subsystem{

    private static Drivetrain instance;

    private SwerveKinematics swerveKinematics = new SwerveKinematics();
    
    public static final double kMaxTranslationSpeed = 15; //feet per second
    public static final double kReversableWheelThreshold = 5;

    public static Drivetrain getInstance(){
        if (instance == null)
            instance = new Drivetrain();
        return instance;
    }

    //min and max voltages for each potentiometer along with the zero angle in degrees.po
    //location 
    public static ModuleConfig frontLeftConfig  = new ModuleConfig(0.20751951, 4.739989749, 155.712);
    public static ModuleConfig frontRightConfig = new ModuleConfig(0.21362302, 4.704589362, 304.221);
    public static ModuleConfig backLeftConfig   = new ModuleConfig(0.26123044, 4.735106937, 194.783);
    public static ModuleConfig backRightConfig  = new ModuleConfig(0.22583005, 4.758300294, 153.676);

    public static PigeonIMU mGyro;
    public volatile double gyroOffset = 0;

    public static SwerveDriveModule mFLmodule;
    public static SwerveDriveModule mFRmodule;
    public static SwerveDriveModule mBLmodule;
    public static SwerveDriveModule mBRmodule;

    private Notifier mHeadingPID;
    private static final double kP = 1/180d;
    private static final double kI = 0;
    private static final double kD = 1/5000d; //70
    public static boolean closedLoopHeadingEnabled;
    private volatile double mLastError = 0;
    private volatile double mSetpoint = 0;
    private volatile double mClosedLoopRotateOutput;
    private static final double kTimestep = 0.02;

    public Position mRobotPos;

    public TalonSRX gyroMotor;
    public VictorSPX flTurn;
    public VictorSPX blTurn;
    public VictorSPX brTurn;

    SwerveDriveModule[] mAllModules; 

    public double startTime = 0;

    private Drivetrain(){
        //location, config, turn motor.
        gyroMotor = new TalonSRX(14);
        flTurn = new VictorSPX(3);
        blTurn = new VictorSPX(2);
        brTurn = new VictorSPX(15);

        mFLmodule = new SwerveDriveModule(SwerveLocation.FRONT_LEFT, frontLeftConfig, flTurn , 4, 1, false);
        mFRmodule = new SwerveDriveModule(SwerveLocation.FRONT_RIGHT, frontRightConfig, gyroMotor , 13, 2, true);
        mBLmodule = new SwerveDriveModule(SwerveLocation.BACK_LEFT, backLeftConfig, blTurn, 1, 0, true);
        mBRmodule = new SwerveDriveModule(SwerveLocation.BACK_RIGHT, backRightConfig, brTurn, 16, 3, false);

        gyroMotor.configOpenloopRamp(0.08);
        gyroMotor.configVoltageCompSaturation(12);
        gyroMotor.enableVoltageCompensation(true);

        flTurn.configOpenloopRamp(0.08);
        flTurn.configVoltageCompSaturation(12);
        flTurn.enableVoltageCompensation(true);

        blTurn.configOpenloopRamp(0.08);
        blTurn.configVoltageCompSaturation(12);
        blTurn.enableVoltageCompensation(true);

        brTurn.configOpenloopRamp(0.08);
        brTurn.configVoltageCompSaturation(12);
        brTurn.enableVoltageCompensation(true);

        mRobotPos = new Position(0, 0);

        mAllModules = new SwerveDriveModule[]{mFLmodule, mFRmodule, mBLmodule, mBRmodule};

        mGyro = new PigeonIMU(gyroMotor);

        //no lag
        mHeadingPID = new Notifier(() -> {
            // startTime = Timer.getFPGATimestamp();

            // double error = boundHalfDegrees(mSetpoint - getAngle());

            // mClosedLoopRotateOutput = (error* -kP) + ((error - mLastError) * -kD * kTimestep);
            // // mClosedLoopRotateOutput = 0;
            // mLastError = error;
        });

        mHeadingPID.startPeriodic(kTimestep);
    }

    @Override
    protected void initDefaultCommand() {}

    public void calibrateAll(){
        for(SwerveDriveModule m : mAllModules){
            m.calibrate();
        }
    }

    public void enableAll(boolean enable){
        SwerveDriveModule.motorEnabled = enable;
    }

    public void displayTurnAngles(){
        // System.out.println(mFLmodule.getCurrent());
        for (SwerveDriveModule m: mAllModules){
            SmartDashboard.putNumber("Swerve " + m.getLocationAsString(), m.getAngle());
        }
    }

    public void displayTurnAnglesRAW(){
        // System.out.println(mFLmodule.getCurrent());
        for (SwerveDriveModule m: mAllModules){
            SmartDashboard.putNumber("Swerve " + m.getLocationAsString(), m.getRawAngle());
        }
    }

    public void setRawAngles(double deg){
        for (SwerveDriveModule m: mAllModules){
            m.setSteeringDegrees(deg);
        }
    }

    public void setRawSpeed(double vel){
        for (SwerveDriveModule m: mAllModules){
            m.setDrivePower(vel);
            // m.debug(Robot.xbox.getRawAxis(4));
        }
    }

    @Deprecated
    public void getSpeeds(double speed){
        for (SwerveDriveModule m: mAllModules){
            System.out.print(m.setRawSpeed(speed) + "\t");
        }
        System.out.println();
    }

    public double getModuleCommandedSpeed(SwerveLocation location){
        switch (location){
            case BACK_LEFT:
                return swerveKinematics.rlWheelSpeed();
            case BACK_RIGHT:
                return swerveKinematics.rrWheelSpeed();
            case FRONT_LEFT:
                return swerveKinematics.flWheelSpeed();
            case FRONT_RIGHT:
                return swerveKinematics.frWheelSpeed();
        }
        return 0;
    }

    public void setRobotRelative(double x, double y, double rotation){
        swerveKinematics.calculate(x, y, rotation);

        if (swerveKinematics.getAverageSpeed() > kReversableWheelThreshold){
            mFLmodule.set(swerveKinematics.flSteeringAngle(), swerveKinematics.flWheelSpeed() * kMaxTranslationSpeed, true);
            mFRmodule.set(swerveKinematics.frSteeringAngle(), swerveKinematics.frWheelSpeed() * kMaxTranslationSpeed, true);
            mBLmodule.set(swerveKinematics.rlSteeringAngle(), swerveKinematics.rlWheelSpeed() * kMaxTranslationSpeed, true);
            mBRmodule.set(swerveKinematics.rrSteeringAngle(), swerveKinematics.rrWheelSpeed() * kMaxTranslationSpeed, true);
        }else{
            mFLmodule.set(swerveKinematics.flSteeringAngle(), swerveKinematics.flWheelSpeed() * kMaxTranslationSpeed, false);
            mFRmodule.set(swerveKinematics.frSteeringAngle(), swerveKinematics.frWheelSpeed() * kMaxTranslationSpeed, false);
            mBLmodule.set(swerveKinematics.rlSteeringAngle(), swerveKinematics.rlWheelSpeed() * kMaxTranslationSpeed, false);
            mBRmodule.set(swerveKinematics.rrSteeringAngle(), swerveKinematics.rrWheelSpeed() * kMaxTranslationSpeed, false);
        }
        
    }

    // public void calibrateGyro(boolean calibrate){
    //     if (calibrate){
    //         mGyro.enterCalibrationMode(CalibrationMode.Accelerometer);
    //         mGyro.enterCalibrationMode(CalibrationMode.BootTareGyroAccel);
    //     }else{

    //     }
    // }

    public void setFieldRelative(double controllerX, double controllerY, double controllerRotate){
        setRobotRelative((controllerX * Math.cos(angleRadians()) - (controllerY * Math.sin(angleRadians()))), 
                  (controllerX * Math.sin(angleRadians()) + (controllerY * Math.cos(angleRadians()))), controllerRotate);
    } 

    public void setOnlyYRobotRelative(double controllerX, double controllerY, double controllerRotate){
        setRobotRelative((controllerX), 
                  (controllerX * Math.sin(angleRadians()) + (controllerY * Math.cos(angleRadians()))), controllerRotate);
    }

    double error = 0;

    public void setFieldRelative(double controllerX, double controllerY, double controllerRotate, boolean closedLoopHeading){
        if (closedLoopHeading){
            setClosedLoopSetpoint(controllerRotate);
            {
                error = boundHalfDegrees(mSetpoint - getAngle());

                mClosedLoopRotateOutput = -((error* kP) + ((error - mLastError) * kD / kTimestep));

                if (Math.abs(mClosedLoopRotateOutput) < 0.02){
                    mClosedLoopRotateOutput = 0;
                }

                mClosedLoopRotateOutput = Math.signum(mClosedLoopRotateOutput) * Math.min(Math.abs(mClosedLoopRotateOutput), 0.6);

                mLastError = error;
            }

            setFieldRelative(controllerX, controllerY, mClosedLoopRotateOutput);
        }else{
            setFieldRelative(controllerX, controllerY, controllerRotate);
        }
    }

    public void setClosedLoopSetpoint(double setpoint){
        mSetpoint = setpoint;
    }

    public double getRawAngle(){
        return mGyro.getFusedHeading();
    }

    public double getAngle(){
        return getRawAngle() - gyroOffset;
    }

    public void zeroGyro(){
        gyroOffset = getRawAngle();
    } 

    public double angleRadians(){
        return Math.toRadians(getAngle());
    }

    //no lag
    public static double boundHalfDegrees(double angle_degrees) {
        while (angle_degrees >= 180.0) angle_degrees -= 360.0;
        while (angle_degrees < -180.0) angle_degrees += 360.0;
        return angle_degrees;
    }

    public static double kSticktoHeading(boolean snap){
        


        return 0;
    }

    public void updatePosition(){
        mFLmodule.updateOdom();
        mBRmodule.updateOdom();
        mFRmodule.updateOdom();
        mBLmodule.updateOdom();

        mRobotPos = new Position(
            mRobotPos.x + ((mFLmodule.getXYOffset().x + mFRmodule.getXYOffset().x + mBLmodule.getXYOffset().x + mBRmodule.getXYOffset().x) / 4.0),
            mRobotPos.y + ((mFLmodule.getXYOffset().y + mFRmodule.getXYOffset().y + mBLmodule.getXYOffset().y + mBRmodule.getXYOffset().y) / 4.0)
        );

        // System.out.println(mRobotPos.y);
        // if (((mFLmodule.getXYOffset().y + mFRmodule.getXYOffset().y + mBLmodule.getXYOffset().y + mBRmodule.getXYOffset().y) / 4.0) != 0)
        // if (mFLmodule.getXYOffset().y != 0)
        //     System.out.println(((mFLmodule.getXYOffset().y)));
    }

    public void zeroPosition(){
        mRobotPos.x = 0;
        mRobotPos.y = 0;
    }

    public Position getPosition(){
        return new Position(mRobotPos);
    }

}