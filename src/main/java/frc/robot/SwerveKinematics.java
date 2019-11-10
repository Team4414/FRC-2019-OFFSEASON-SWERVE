package frc.robot;

public class SwerveKinematics {
	
	private double frontRightWheelSpeed = 0.0;
	private double frontLeftWheelSpeed = 0.0;
	private double rearLeftWheelSpeed = 0.0;
	private double rearRightWheelSpeed = 0.0;
	
	private double frontRightSteeringAngle = 0.0;
	private double frontLeftSteeringAngle = 0.0;
	private double rearLeftSteeringAngle = 0.0;
	private double rearRightSteeringAngle = 0.0;
	
	public double[] wheelAngles = new double[4];
	public double[] wheelSpeeds = new double[4];
	
	public void calculate(double x, double y, double rotate){
		double A = x - rotate * (Robot.kWheelBaseLength / Robot.kSwerveDiagonal);
	    double B = x + rotate * (Robot.kWheelBaseLength / Robot.kSwerveDiagonal);
	    double C = y - rotate * (Robot.kWheelBaseWidth / Robot.kSwerveDiagonal);
	    double D = y + rotate * (Robot.kWheelBaseWidth / Robot.kSwerveDiagonal);
	    
	    frontRightWheelSpeed = Math.sqrt((B * B) + (C * C));
	    frontLeftWheelSpeed  = Math.sqrt((B * B) + (D * D));
	    rearLeftWheelSpeed   = Math.sqrt((A * A) + (D * D));
	    rearRightWheelSpeed  = Math.sqrt((A * A) + (C * C));
	    double max = frontRightWheelSpeed;
	    max = normalize(max, frontLeftWheelSpeed);
	    max = normalize(max, rearLeftWheelSpeed);
	    max = normalize(max, rearRightWheelSpeed);
	    if(max > 1.0){
	    	frontRightWheelSpeed /= max;
	        frontLeftWheelSpeed /= max;
	        rearLeftWheelSpeed /= max;
	        rearRightWheelSpeed /= max;
	    }
	    
	    wheelSpeeds[0] = frontRightWheelSpeed;
	    wheelSpeeds[1] = frontLeftWheelSpeed;
	    wheelSpeeds[2] = rearLeftWheelSpeed;
	    wheelSpeeds[3] = rearRightWheelSpeed;
	    
	    frontRightSteeringAngle = Math.atan2(B, C)*180/Math.PI; 
	    frontLeftSteeringAngle = Math.atan2(B, D)*180/Math.PI;
	    rearLeftSteeringAngle = Math.atan2(A, D)*180/Math.PI;
	    rearRightSteeringAngle = Math.atan2(A, C)*180/Math.PI;
	    
	    wheelAngles[0] = frontRightSteeringAngle;
	    wheelAngles[1] = frontLeftSteeringAngle;
	    wheelAngles[2] = rearLeftSteeringAngle;
	    wheelAngles[3] = rearRightSteeringAngle;
	}
	
	public double frWheelSpeed(){
		return frontRightWheelSpeed;
	}
	public double flWheelSpeed(){
		return frontLeftWheelSpeed;
	}
	public double rlWheelSpeed(){
		return rearLeftWheelSpeed;
	}
	public double rrWheelSpeed(){
		return rearRightWheelSpeed;
	}
	
	public double frSteeringAngle(){
		return frontRightSteeringAngle;
	}
	public double flSteeringAngle(){
		return frontLeftSteeringAngle;
	}
	public double rlSteeringAngle(){
		return rearLeftSteeringAngle;
	}
	public double rrSteeringAngle(){
		return rearRightSteeringAngle;
	}
	
	public double getAverageSpeed(){
		return (frontLeftWheelSpeed + frontRightWheelSpeed + rearLeftWheelSpeed + rearRightWheelSpeed) / 4d;
	}
    
    public static double normalize(double current, double test){
    	if(current > test) return current;
    	return test;
    }
}