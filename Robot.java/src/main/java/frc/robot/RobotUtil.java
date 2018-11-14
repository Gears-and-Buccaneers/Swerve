package frc.robot;

public class RobotUtil {
	double initialForwardTime = 300;
	double toSwitchTime = 2000;
	double turn90Time = 1250;
	double centerToSideTime = 1200;
	double sideToSwitchTime = 300;
	double liftToSwitchTime = 800;
	double spitOutTime = 400;
	double pastSwitchTime = 800;
	double crossBehindSwitchTime = 2000;
	double backOfSwitchApproachTime = 400;
	double restTime = 15000;
	// It's just borrowing some values.
	double[] goUpToSwitch = {0, 0, .7, 0};
	double[] goForward = {.7,.7,.1,0};
	double[] goBackward = {-.7,-.7,0,0};
	double[] goSlowForward = {.55,.55,.1,0};
	double[] goSpitOut = {0,0,.1,1};
	double[] STOP = {0,0,0,0};
	
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
	
	//-1 is left, 1 is right
	public double[] turn(int dir) {
		return new double[] {dir*.55,dir*-.55,0,0};
	}
}
