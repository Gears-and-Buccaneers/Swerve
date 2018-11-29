package frc.robot;

public class RobotUtil {
	
	public int pos = -1;
	
    public double stickToT(double axis) {
    	axis *=-1;
    	double difference = 1-axis;
    	return 1 - .2*difference;
    }
	
	public double normalize(double x)
    {
    	if (x >= 1) return 1;
    	else if (x <= -1) return -1;
    	else return x*Math.abs(x);
    }

}
