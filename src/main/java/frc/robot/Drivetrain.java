package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.SwerveDriveModule.ModuleConfig;
import frc.robot.SwerveDriveModule.SwerveLocation;

public class Drivetrain extends Subsystem{

    private static Drivetrain instance;

    private SwerveKinematics swerveKinematics = new SwerveKinematics();
    
    public static final double kMaxTranslationSpeed = 8; //feet per second

    public static Drivetrain getInstance(){
        if (instance == null)
            instance = new Drivetrain();
        return instance;
    }

    //min and max voltages for each potentiometer along with the zero angle in degrees.
    public static ModuleConfig frontLeftConfig  = new ModuleConfig(0.20751951, 4.739989749, 336.73040);
    public static ModuleConfig frontRightConfig = new ModuleConfig(0.21362302, 4.704589362, 303.93040);
    public static ModuleConfig backLeftConfig   = new ModuleConfig(0.26123044, 4.735106937, 193.80081);
    public static ModuleConfig backRightConfig  = new ModuleConfig(0.22583005, 4.758300294, 153.28844);

    public static PigeonIMU mGyro;
    public double gyroOffset = 0;

    public static SwerveDriveModule mFLmodule;
    public static SwerveDriveModule mFRmodule;
    public static SwerveDriveModule mBLmodule;
    public static SwerveDriveModule mBRmodule;

    public TalonSRX gyroMotor;

    SwerveDriveModule[] mAllModules; 

    private Drivetrain(){
        gyroMotor = new TalonSRX(14);
        mFLmodule = new SwerveDriveModule(SwerveLocation.FRONT_LEFT, frontLeftConfig, new VictorSPX(3) , 4, 1, false);
        mFRmodule = new SwerveDriveModule(SwerveLocation.FRONT_RIGHT, frontRightConfig, gyroMotor , 13, 2, true);
        mBLmodule = new SwerveDriveModule(SwerveLocation.BACK_LEFT, backLeftConfig, new VictorSPX(2), 1, 0, true);
        mBRmodule = new SwerveDriveModule(SwerveLocation.BACK_RIGHT, backRightConfig, new VictorSPX(15), 16, 3, false);

        mAllModules = new SwerveDriveModule[]{mFLmodule, mFRmodule, mBLmodule, mBRmodule};

        mGyro = new PigeonIMU(gyroMotor);
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
        }
    }

    public void setRobotRelative(double x, double y, double rotation){
        swerveKinematics.calculate(x, y, rotation);
        
        mFLmodule.set(swerveKinematics.flSteeringAngle(), swerveKinematics.flWheelSpeed() * kMaxTranslationSpeed);
        mFRmodule.set(swerveKinematics.frSteeringAngle(), swerveKinematics.frWheelSpeed() * kMaxTranslationSpeed);
        mBLmodule.set(swerveKinematics.rlSteeringAngle(), swerveKinematics.rlWheelSpeed() * kMaxTranslationSpeed);
        mBRmodule.set(swerveKinematics.rrSteeringAngle(), swerveKinematics.rrWheelSpeed() * kMaxTranslationSpeed);
    }

    public void setFieldRelative(double controllerX, double controllerY, double controllerRotate){
        setRobotRelative((controllerX * Math.cos(angleRadians()) - (controllerY * Math.sin(angleRadians()))), 
                  (controllerX * Math.sin(angleRadians()) + (controllerY * Math.cos(angleRadians()))), controllerRotate);
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

}