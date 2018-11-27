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

		frontLeftEncoder = new Encoder(6, 7, true, Encoder.EncodingType.k4X);
		


		frontRightEncoder = new Encoder(8, 9, false, Encoder.EncodingType.k4X);
		
		backLeftEncoder = new Encoder(2, 3, false, Encoder.EncodingType.k4X);
		frontLeftEncoder.setMaxPeriod(.1);
		frontLeftEncoder.setMinRate(10);
		frontLeftEncoder.setDistancePerPulse((360/7)*1.2);
		frontLeftEncoder.setReverseDirection(true);
		frontLeftEncoder.setSamplesToAverage(7);

		backRightEncoder = new Encoder(4, 5, false, Encoder.EncodingType.k4X);
		frontLeftEncoder.setMaxPeriod(.1);
		frontLeftEncoder.setMinRate(10);
		frontLeftEncoder.setDistancePerPulse((360/7)*1.2);
		frontLeftEncoder.setReverseDirection(true);
		frontLeftEncoder.setSamplesToAverage(7);
    }
    
    public void teleopInit(){
		frontLeftEncoder.reset();

		frontLeftEncoder.setMaxPeriod(.1);
		frontLeftEncoder.setMinRate(10);
		frontLeftEncoder.setDistancePerPulse(1/1.2);
		//frontLeftEncoder.setDistancePerCount(1/1.2);
		frontLeftEncoder.setReverseDirection(true);
		//frontLeftEncoder.setSamplesToAverage(7);

		

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
		SmartDashboard.putNumber("Count", frontLeftEncoder.get());
		SmartDashboard.putNumber("Rotation", ahrs.getYaw());	   
	}
    
    public void testPeriodic() {

    }
    
    public void autonomousInit() {
	}

	public void autonomousPeriodic(){
	}
}
