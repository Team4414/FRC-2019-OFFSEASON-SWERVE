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

    public static Drivetrain getInstance(){
        if (instance == null)
            instance = new Drivetrain();
        return instance;
    }

    public static final ModuleConfig frontLeftConfig = new ModuleConfig(0.20751951, 4.739989749, 336.7304066792351);
    public static ModuleConfig frontRightConfig = new ModuleConfig(0.213623025, 4.704589362 , 303.9304158738788);
    public static ModuleConfig backLeftConfig = new ModuleConfig(0.261230442, 4.735106937, 193.80081);
    public static ModuleConfig backRightConfig = new ModuleConfig(0.225830055, 4.7583002940, 153.288446);

    public static PigeonIMU mGyro;
    public double gyroZero = 0;

    // public static ModuleConfig frontLeftConfig = new ModuleConfig();
    // public static ModuleConfig frontRightConfig = new ModuleConfig();
    // public static ModuleConfig backLeftConfig = new ModuleConfig();
    // public static ModuleConfig backRightConfig = new ModuleConfig();

    //turn, drive, sensor

    public static SwerveDriveModule mFLmodule;
    public static SwerveDriveModule mFRmodule;
    public static SwerveDriveModule mBLmodule;
    public static SwerveDriveModule mBRmodule;

    public TalonSRX gyroMotor;

    SwerveDriveModule[] mAllModules; 

    private Drivetrain(){
        gyroMotor = new TalonSRX(14);
        mFLmodule = new SwerveDriveModule(SwerveLocation.FRONT_LEFT, frontLeftConfig, new VictorSPX(3) , 4, 1, false, false);
        mFRmodule = new SwerveDriveModule(SwerveLocation.FRONT_RIGHT, frontRightConfig, gyroMotor , 13, 2, true, false);
        mBLmodule = new SwerveDriveModule(SwerveLocation.BACK_LEFT, backLeftConfig, new VictorSPX(2), 1, 0, true, false);
        mBRmodule = new SwerveDriveModule(SwerveLocation.BACK_RIGHT, backRightConfig, new VictorSPX(15), 16, 3, false, false);

        mAllModules = new SwerveDriveModule[]{mFLmodule, mFRmodule, mBLmodule, mBRmodule};

        // mGyro = new PigeonIMU(Drivetrain.mFRmodule.getTurnTalon());
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
        for (SwerveDriveModule m : mAllModules){
            m.motorEnabled = enable;
        }
    }
    
    public void zeroAll(){
        for (SwerveDriveModule m : mAllModules){
            m.zero();
        }
    }

    public void pushTurnAngles(){
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

    public void setSpeeds(double x, double y, double rotation){
        swerveKinematics.calculate(x, y, rotation);
        
        mFLmodule.set(swerveKinematics.flSteeringAngle(), swerveKinematics.flWheelSpeed());
        mFRmodule.set(swerveKinematics.frSteeringAngle(), swerveKinematics.frWheelSpeed());
        mBLmodule.set(swerveKinematics.rlSteeringAngle(), swerveKinematics.rlWheelSpeed());
        mBRmodule.set(swerveKinematics.rrSteeringAngle(), swerveKinematics.rrWheelSpeed());
    }

    public double getRawAngle(){
        // System.out.println(getAngle());
        return mGyro.getFusedHeading();
    }

    public double getAngle(){
        return getRawAngle() - gyroZero;
    }

    public void zeroGyro(){
        gyroZero = getRawAngle();
    }

    public void controlFieldRelative(double controllerX, double controllerY, double controllerRotate){
        setSpeeds((controllerX * Math.cos(angleRadians()) - (controllerY * Math.sin(angleRadians()))), 
                  (controllerX * Math.sin(angleRadians()) + (controllerY * Math.cos(angleRadians()))), controllerRotate);
    }  

    public double angleRadians(){
        return Math.toRadians(getAngle());
    }

}