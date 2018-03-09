package org.usfirst.frc.team3495.robot;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	final String defaultAuto = "Default";
	final String customAuto = "My Auto";
	String autoSelected;
	SendableChooser<String> chooser = new SendableChooser<>();
	CANTalon d3 = new CANTalon(3);
	CANTalon d0 = new CANTalon(0);
	CANTalon d5 = new CANTalon(5); 
	CANTalon d8 = new CANTalon (8);

	Joystick left = new Joystick(1);
	Joystick right = new Joystick(2);
	Joystick controller = new Joystick(0);

	Encoder enc1 = new Encoder(2, 3, false);
	//Encoder enc2 = new Encoder(4, 5, false);
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		chooser.addDefault("Default Auto", defaultAuto);
		chooser.addObject("My Auto", customAuto);
		SmartDashboard.putData("Auto choices", chooser);
		enc1.setDistancePerPulse(.024);
		//	enc2.setDistancePerPulse(.048);


	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomousInit() {
		autoSelected = chooser.getSelected();
		enc1.setDistancePerPulse(.024);
		//enc2.setDistancePerPulse(.2617);
		// autoSelected = SmartDashboard.getString("Auto Selector",
		// defaultAuto);
		System.out.println("Auto selected: " + autoSelected);
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		switch (autoSelected) {
		case customAuto:
			// Put custom auto code here
			break;
		case defaultAuto:
		default:
			// Put default auto code here
			break;
		}
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		enc1.startLiveWindowMode();
		enc1.setDistancePerPulse(.024);
		System.out.println("Encoder Value: " + enc1.getDistance());
		//System.out.println("Encoder Value 2: " + enc2.getDistance());

		tankDrive(left.getY(), right.getY());
		//double leftjoystickdiag;
		//double rightjoystickdiag;
		//leftjoystickdiag = left.getDirectionDegrees();
		//rightjoystickdiag = right.getDirectionDegrees();
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
	}
	public void tankDrive(double left, double right){
		d3.set(-left);
		d5.set(-left);
		d0.set(right);
		d8.set(right);
	}
} 


