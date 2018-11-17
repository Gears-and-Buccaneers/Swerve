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

		Encoder frontLeftEncoder = new Encoder(2, 3, false, Encoder.EncodingType.k4X);
		frontLeftEncoder.setMaxPeriod(.1);
		frontLeftEncoder.setMinRate(10);
		frontLeftEncoder.setDistancePerPulse(5);
		frontLeftEncoder.setReverseDirection(true);
		frontLeftEncoder.setSamplesToAverage(7);

		Encoder frontRightEncoder = new Encoder(0, 1, true, Encoder.EncodingType.k4X);
		frontRightEncoder.setMaxPeriod(.1);
		frontRightEncoder.setMinRate(10);
		frontRightEncoder.setDistancePerPulse(5);
		frontRightEncoder.setReverseDirection(true);
		frontRightEncoder.setSamplesToAverage(7);

		Encoder backLeftEncoder = new Encoder(6, 7, false, Encoder.EncodingType.k4X);
		backLeftEncoder.setMaxPeriod(.1);
		backLeftEncoder.setMinRate(10);
		backLeftEncoder.setDistancePerPulse(5);
		backLeftEncoder.setReverseDirection(true);
		backLeftEncoder.setSamplesToAverage(7);

		Encoder backRightEncoder = new Encoder(4, 5, true, Encoder.EncodingType.k4X);
		backRightEncoder.setMaxPeriod(.1);
		backRightEncoder.setMinRate(10);
		backRightEncoder.setDistancePerPulse(5);
		backRightEncoder.setReverseDirection(true);
		backRightEncoder.setSamplesToAverage(7);
    }
    
    public void teleopInit(){
    }
    		  
    public void teleopPeriodic() {
		 SmartDashboard.putNumber("Rotation", ahrs.getYaw());	   
	}
    
    public void testPeriodic() {
		SmartDashboard.putNumber("Rotation", ahrs.getYaw());
		SmartDashboard.putNumber("Encoder FL Position", frontLeftEncoder.getPosition());
		SmartDashboard.putNumber("Encoder BR Position", backRightEncoder.getPosition());
		SmartDashboard.putNumber("Encoder FR Position", frontRightEncoder.getPosition());
		SmartDashboard.putNumber("Encoder BL Position", backLeftEncoder.getPosition());

		frontRightDriver.set(.3);
		frontLeftDriver.set(PO, .3);
		backLeftDriver.set(.3);
		backRightDriver.set(.3);

		frontRightRotator.set(PO, .4);
		frontLeftRotator.set(PO, -.4);
		backLeftRotator.set(PO, -.4);
		backRightRotator.set(PO, .4);

    }
    
    public void autonomousInit() {
	}

	public void autonomousPeriodic(){
	}
}
