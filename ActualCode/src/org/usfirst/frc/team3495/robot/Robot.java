
package org.usfirst.frc.team3495.robot;

//import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Set;

import org.usfirst.frc.team3495.robot.commands.ExampleCommand;
import org.usfirst.frc.team3495.robot.subsystems.ExampleSubsystem;

import com.ctre.CANTalon;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {

	public static final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();
	public static OI oi;

	Command autonomousCommand;
	SendableChooser<Command> chooser = new SendableChooser<>();

	NetworkTable camera = NetworkTable.getTable("GRIP/contours");
	NetworkTable table = NetworkTable.getTable("SmartDashboard");

//	AnalogPotentiometer pot = new AnalogPotentiometer(0);
	
	
	
	double[] area;
	double[] x;

	boolean turnLeft;
	boolean turnRight;
	boolean center;
	boolean aligned = false;

	CANTalon intakem = new CANTalon(1);
	CANTalon shooter1 = new CANTalon(6);
	CANTalon d3 = new CANTalon(3);
	CANTalon d0 = new CANTalon(0);
	CANTalon shooter2 = new CANTalon(7);
	CANTalon d5 = new CANTalon(5); 
	CANTalon index = new CANTalon(4);
	CANTalon climbert = new CANTalon(2);
	CANTalon d8 = new CANTalon (8);
	Talon screw = new Talon(9);

	Joystick left = new Joystick(1);
	Joystick right = new Joystick(2);
	Joystick controller = new Joystick(0);

	double lastCheck = 0;
	boolean firstRun = true;

	JoystickButton climberb = new JoystickButton(controller, 6); //Turns on climber motor
	//JoystickButton flyb = new JoystickButton(controller, 2); //Turns on shooter1 & shooter2 motors, reverses cork
	JoystickButton pistons1 = new JoystickButton(controller, 5); //Activates pistons?
	JoystickButton intakeB = new JoystickButton(controller, 2);//Turns on intake motors
	JoystickButton gateI = new JoystickButton(controller, 1);//Reverses Intake
	//JoystickButton shooterb = new JoystickButton(controller, 4);//ShooterIntake
	JoystickButton comp = new JoystickButton(controller, 7);
	JoystickButton reverseI = new JoystickButton(controller, 3);

	DoubleSolenoid small = new DoubleSolenoid(1, 4, 5);//good
	DoubleSolenoid big = new DoubleSolenoid(1, 2, 3); //bad
	DoubleSolenoid intakeG = new DoubleSolenoid(1, 0, 1);

	String currentAuton;

	boolean bool = true;
	boolean bool1 = true;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		oi = new OI();
		chooser.addDefault("Default Auto", new ExampleCommand());
		// chooser.addObject("My Auto", new MyAutoCommand());
		SmartDashboard.putData("Auto mode", chooser);
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {

	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString code to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons
	 * to the switch structure below with additional strings & commands.
	 */
	@Override
	public void autonomousInit() {
		autonomousCommand = chooser.getSelected();
		currentAuton = table.getString("Auto Selector");

		switch(currentAuton){
		case "gear":
			break;
		case "boiler":

			break;
		}


		/*
		 * String autoSelected = SmartDashboard.getString("Auto Selector",
		 * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
		 * = new MyAutoCommand(); break; case "Default Auto": default:
		 * autonomousCommand = new ExampleCommand(); break; }
		 */

		// schedule the autonomous command (example)
		if (autonomousCommand != null)
			autonomousCommand.start();
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		bool = true;
		small.set(Value.kReverse);
		big.set(Value.kReverse);

		if (autonomousCommand != null)
			autonomousCommand.cancel();
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();		

		tankDrive(left.getY(), right.getY());

		

		table.putNumber("Trigger", controller.getThrottle());
		table.putNumber("Trigger 2?", controller.getTwist());

		intake();

		//		if(pistons1.get()){
		//			small.set(Value.kForward);
		//		}else{
		//			small.set(Value.kReverse);
		//		}
		//		if(pistons2.get()){
		//			big.set(Value.kForward);
		//		}else{
		//			big.set(Value.kReverse);
		//		}

		shooter();
		climber();
		//gear();
		//		piston();

	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		LiveWindow.run();
		if(comp.get()){
			index.set(.8);
		}else{
			index.set(0);
		}
	}																	

	public void tankDrive(double left, double right){
		d3.set(-left);
		d5.set(-left);
		d0.set(right);
		d8.set(right);
	}	


	//	public void piston(){
	//		if(pistons1.get()){
	//			small.set(Value.kForward);
	//			small.set(Value.kReverse);
	//		}
	//		if(pistons2.get()){
	//			big.set(Value.kForward);
	//			big.set(Value.kReverse);
	//
	//		}
	//
	//	}

	
	public void intake(){
		if(bool){
			if(pistons1.get()){
				big.set(Value.kForward);
				Timer.delay(.5);
				small.set(Value.kForward);
				bool = false;
			}
		}else{
			if(pistons1.get()){
				small.set(Value.kReverse);
				Timer.delay(.5);
				big.set(Value.kReverse);
				bool = true;
			}else if(gateI.get()){
				intakeG.set(Value.kReverse);
			}else{
				small.set(Value.kForward);
				intakeG.set(Value.kForward);
			}
		}
		
		if(!bool){
			if(intakeB.get()){
				intakem.set(1);
				index.set(-5);
			}else if(controller.getTwist() >= .8){
				intakem.set(.75);
				screw.set(-.4);
				index.set(.85);
			}else if(reverseI.get()){
				intakem.set(-.5);
			}else if(gateI.get()){
				intakem.set(1);
				index.set(-.5);
			}else{
				intakem.set(0);	
				index.set(0);
				screw.set(0);
			}
		}else{
			intakem.set(0);
			index.set(0);
			screw.set(0);
			intakeG.set(Value.kReverse);
		}


	}


	/*	public void intake(){

		if (pistons1.get()){
			if (bool){
				big.set(Value.kForward);
				Timer.delay(1);
				small.set(Value.kReverse);
				intakem.set(1);
				corkscrew.set(1);
				bool = false;
			}else{
				intakem.set(0);
				corkscrew.set(0);
				bool = true;
			}
		}


		if (pistons2.get()){
			if (bool){
				Timer.delay(1);
				small.set(Value.kForward);
				bool = false;
			}else{
				Timer.delay(1);
				small.set(Value.kReverse);
				bool = true;
			}
		}
	}
	 */



	public void shooter(){
		if (controller.getThrottle() >= .8){
			shooter1.set(.61);
			shooter2.set(-.61);
		}else{
			shooter1.set(0);
			shooter2.set(0);
		}
	}


	public void climber(){
		if (climberb.get()){
			climbert.set(1);
		}else{
			climbert.set(0);
		}
		/*
		if(!firstRun && Math.abs(climbert.getOutputCurrent() - lastCheck) > 5){
			climbert.set(0);
		}else{
			lastCheck = climbert.getOutputCurrent();
			firstRun = false;
		}
		 */

	}
	
	public void gearSystem(){
		
	}
	/*
	public void gear(){
		if (gearb.get()){
			small.set(Value.kForward);
			Timer.delay(.5);
			big.set(Value.kReverse);
			Timer.delay(.5);
			//gearp.set(Value.kForward);
		}else{
			//gearp.set(Value.kReverse);

		}
	}
	 */
	void camerArea(){
		if(camera.getNumberArray("area") != null){
			area = camera.getNumberArray("area");	
		}
	}

	void cameraAlign(double minX, double maxX){
		if(camera.getNumberArray("centerX") != null){
			x = camera.getNumberArray("centerX");

		}
		if(x != null && x.length != 0){
			if(!aligned && x[x.length - 1] < minX){
				//				tankDrive(-.2, .2);
				turnRight = true;
				turnLeft = false;
				center = false;
			}else if(!aligned && x[x.length - 1] > maxX){
				//				tankDrive(.2, -.2);
				turnRight = false;
				turnLeft = true;
				center = false;
			}else if(x[x.length - 1] > minX && x[x.length - 1] < maxX){
				turnRight = false;
				turnLeft = false;
				center = true;

			}	
		}
	}


}

