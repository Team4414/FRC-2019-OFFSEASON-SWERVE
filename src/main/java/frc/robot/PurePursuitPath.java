package frc.robot;

import edu.wpi.first.wpilibj.command.Command;
import jaci.pathfinder.Trajectory;

public class PurePursuitPath extends Command{
    public static final int kLookaheadSegments = 15;
    public static final double kVelocityDeadzone = 0;
    
    public static final double kFeet2Meters = 3.28084;
    public static final double kMeters2Feet = 1 / kFeet2Meters;

    public Trajectory mTraj;

    public PurePursuitPath(Trajectory traj){
        mTraj = traj;
        System.out.println(mTraj.get(0).x);
    }

    double gX, gY, rX, rY;
    double vectorVel, theta;

    int index;

    @Override
    protected void initialize() {
        index = 0;
        System.out.println("Initalized");
    }

    @Override
    protected void execute() {
        if (mTraj.length() <= (index + kLookaheadSegments)){
            gX = mTraj.get(mTraj.length() - 1).x;
            gY = -mTraj.get(mTraj.length() - 1).y;
        }else{
            gX = mTraj.get(index + kLookaheadSegments).x;
            gY = -mTraj.get(index + kLookaheadSegments).y;
        }

        rX = Drivetrain.getInstance().getPosition().x;
        rY = Drivetrain.getInstance().getPosition().y;

        if (Math.abs(gY - rY) < 0.2 || Math.abs(gX - rX) < 0.2){
            // theta = 0;
        }else{
            theta = Math.atan2( (gY - rY) , (gX - rX) );
        }

        System.out.println("XError: " + (gX-rX) + "\tYError: " + (gY-rY) + "\tTheta: " + theta + "\tModuleVelError: " + Drivetrain.getInstance().mFLmodule.getSpeed());

        if (Math.abs(mTraj.get(index).velocity) > kVelocityDeadzone){
            Drivetrain.getInstance().setFieldRelativeRawVel(mTraj.get(index).velocity * Math.cos(theta), mTraj.get(index).velocity * Math.sin(theta), 0);
        }else{
            Drivetrain.getInstance().setFieldRelativeRawVel(0, 0, 0);
        }

        index++;
    }

    @Override
    protected boolean isFinished() {
        return (index >= mTraj.length() - 1);
    }



}