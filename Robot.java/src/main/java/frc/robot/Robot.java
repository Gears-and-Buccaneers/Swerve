package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import java.util.ArrayList;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.CameraServer;
import com.kauailabs.navx.frc.AHRS;
//import com.kauailabs.navx.frc.AHRS.BoardAxis;
import edu.wpi.first.wpilibj.SerialPort;



public class Robot extends IterativeRobot {
	
	final ControlMode PO = ControlMode.PercentOutput;
	final ControlMode Pos = ControlMode.Position;
    
	Joystick pad;
    TalonSRX frontRightRotator,backLeftRotator, backRightRotator; 
	Talon  frontRightDriver, backRightDriver, backLeftDriver;
	Encoder frontRightEncoder, backLeftEncoder, backRightEncoder;

	SwerveSystem flss;
	
	RobotUtil ru = new RobotUtil();

	//AHRS ahrs = new AHRS();
	AHRS ahrs = new AHRS(SerialPort.Port.kUSB1);
	//AHRS ahrs = new AHRS(SerialPort.Port.kMXP, SerialDataType.kProcessedData, (byte)50);
    
    public void robotInit(){
    	
    	//CameraServer.getInstance().startAutomaticCapture();
    	
    	pad = new Joystick(1);
		
		frontRightRotator = new TalonSRX(6);
		backLeftRotator = new TalonSRX(5);
		backRightRotator = new TalonSRX(7);

		frontRightDriver = new Talon(1);
		backRightDriver = new Talon(0);
		backLeftDriver = new Talon(2);

		frontRightEncoder = new Encoder(8, 9, true, Encoder.EncodingType.k4X);
		backRightEncoder = new Encoder(4, 5, false, Encoder.EncodingType.k4X);
		backLeftEncoder = new Encoder(2, 3, true, Encoder.EncodingType.k4X);
    	 
		double power = 0;
		
    	// Example SRX : leftWheelTalon.set(PO,power);
		
		frontRightRotator.set(PO, power);
		backLeftRotator.set(PO, power);
		backRightRotator.set(PO, power);

		frontRightDriver.set(power);
		backLeftDriver.set(power);
		backRightDriver.set(power);

    }
    
    public void teleopInit(){
		frontRightEncoder.reset();
		backLeftEncoder.reset();
		backRightEncoder.reset();
		flss = new SwerveSystem(3,4,6,true);
		SmartDashboard.putNumber("Initial Position", flss.getPos());
    }
    		  
    public void teleopPeriodic() {
		double y = -1*pad.getRawAxis(1);
		double x = pad.getRawAxis(0);
		SmartDashboard.putNumber("Angle", Math.atan(y/x)*180/Math.PI);
		flss.rotateTo(Math.atan(y/x)*180/Math.PI);
		//frss.rotateTo(Math.atan(y/x));
		//blss.rotateTo(Math.atan(y/x));
		//brss.rotateTo(Math.atan(y/x));

		SmartDashboard.putNumber("Important Rotation", flss.getPos());
		//SmartDashboard.putNumber("Front Right Encoder Position", frontRightEncoder.getDistance());
		//SmartDashboard.putNumber("Back Left Encoder Position", backLeftEncoder.getDistance());
		//SmartDashboard.putNumber("Back Right Encoder Position", backRightEncoder.getDistance());
		SmartDashboard.putNumber("Rotation", ahrs.getYaw());	   
	}
    
    public void testPeriodic() {
    }
    
    public void autonomousInit() {
	}

	public void autonomousPeriodic(){
	}
}
