package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

public class SwerveStick{
    private final int mXaxis;
    private final int mYaxis;
    private final Joystick mJoystick;

    private static final double kJoystickDeadzone = 0.05;

    public enum TYPE{
        TRANSLATION, ROTATION;
    }

    public enum AXIS{
        X, Y;
    }

    public SwerveStick(int xAxis, int yAxis, Joystick joystick){
        mXaxis = xAxis;
        mYaxis = yAxis;
        
        mJoystick = joystick;
    }

    public double getHypotenuse(){
        return Math.hypot(mJoystick.getRawAxis(mXaxis), mJoystick.getRawAxis(mYaxis));
    }

    public boolean inDeadzone(){
        return (getHypotenuse() < kJoystickDeadzone);
    }

    public double getAxis(AXIS axis){
        if (inDeadzone()){
            return 0;
        }

        if (axis == AXIS.X){
            return mJoystick.getRawAxis(mXaxis);
        }else{
            return mJoystick.getRawAxis(mYaxis);
        }
    }
}