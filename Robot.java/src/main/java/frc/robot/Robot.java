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
    
	Joystick pad;
    TalonSRX frontLeftRotator, frontRightRotator,backLeftRotator, backRightRotator, frontLeftDriver; 
	Talon  frontRightDriver, backRightDriver, backLeftDriver;
	Encoder frontLeftEncoder, frontRightEncoder, backLeftEncoder, backRightEncoder;
	
	RobotUtil ru = new RobotUtil();

	//AHRS ahrs = new AHRS();
	AHRS ahrs = new AHRS(SerialPort.Port.kUSB1);
	//AHRS ahrs = new AHRS(SerialPort.Port.kMXP, SerialDataType.kProcessedData, (byte)50);
    
    public void robotInit(){
    	
    	//CameraServer.getInstance().startAutomaticCapture();
    	
    	pad = new Joystick(0);
		
		frontLeftRotator = new TalonSRX(4);
		frontRightRotator = new TalonSRX(6);
		backLeftRotator = new TalonSRX(5);
		backRightRotator = new TalonSRX(7);

		frontRightDriver = new Talon(1);
		frontLeftDriver = new TalonSRX(3);
		backRightDriver = new Talon(0);
		backLeftDriver = new Talon(2);

		frontLeftEncoder = new Encoder(6, 7, true, Encoder.EncodingType.k4X);
		frontRightEncoder = new Encoder(8, 9, false, Encoder.EncodingType.k4X);
		backRightEncoder = new Encoder(4, 5, false, Encoder.EncodingType.k4X);
    	 
		double power = 0;
		
    	// Example SRX : leftWheelTalon.set(PO,power);
		
		frontLeftRotator.set(PO, power);
		frontRightRotator.set(PO, power);
		backLeftRotator.set(PO, power);
		backRightRotator.set(PO, power);

		frontRightDriver.set(power);
		frontLeftDriver.set(PO, power);
		backLeftDriver.set(power);
		backRightDriver.set(power);

    }
    
    public void teleopInit(){
		frontLeftEncoder.reset();
		frontRightEncoder.reset();
		backLeftEncoder.reset();
		backRightEncoder.reset();

		frontLeftEncoder.setMaxPeriod(.1);
		frontLeftEncoder.setMinRate(10);
		frontLeftEncoder.setDistancePerPulse(1/1.2);
		frontLeftEncoder.setReverseDirection(true);
		frontLeftEncoder.setSamplesToAverage(7);

		frontRightEncoder.setMaxPeriod(.1);
		frontRightEncoder.setMinRate(10);
		frontRightEncoder.setDistancePerPulse(1/1.2);
		frontRightEncoder.setReverseDirection(true);
		frontRightEncoder.setSamplesToAverage(7);

		backLeftEncoder.setMaxPeriod(.1);
		backLeftEncoder.setMinRate(10);
		backLeftEncoder.setDistancePerPulse(1/1.2);
		backLeftEncoder.setReverseDirection(true);
		backLeftEncoder.setSamplesToAverage(7);

		backRightEncoder.setMaxPeriod(.1);
		backRightEncoder.setMinRate(10);
		backRightEncoder.setDistancePerPulse(1/1.2);
		backRightEncoder.setReverseDirection(true);
		backRightEncoder.setSamplesToAverage(7);

		

    }
    		  
    public void teleopPeriodic() {
		// frontRightDriver.set(.3);
		// frontLeftDriver.set(PO, .3);
		// backLeftDriver.set(.3);
		// backRightDriver.set(.3);

		// frontRightRotator.set(PO, .4);
		// frontLeftRotator.set(PO, -.4);
		// backLeftRotator.set(PO, -.4);
		// backRightRotator.set(PO, .4);
		SmartDashboard.putNumber("Front Left Encoder Position", frontLeftEncoder.getDistance());
		SmartDashboard.putNumber("Front Right Encoder Position", frontRightEncoder.getDistance());
		SmartDashboard.putNumber("Back Left Encoder Position", backLeftEncoder.getDistance());
		SmartDashboard.putNumber("Back Right Encoder Position", backRightEncoder.getDistance());
		SmartDashboard.putNumber("Rotation", ahrs.getYaw());	   
	}
    
    public void testPeriodic() {
    }
    
    public void autonomousInit() {
	}

	public void autonomousPeriodic(){
	}
}
