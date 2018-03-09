/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team3495.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.command.Subsystem;
/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */

public class Robot extends IterativeRobot {
	private static final String kDefaultAuto = "Default";
	private static final String kCustomAuto = "My Auto";
	private String m_autoSelected;
	private SendableChooser<String> m_chooser = new SendableChooser<>();
	TalonSRX t0 = new TalonSRX(0);
	TalonSRX t1 = new TalonSRX(1);
	TalonSRX t2 = new TalonSRX(5);
	TalonSRX t3 = new TalonSRX(6);
	TalonSRX armLeft = new TalonSRX(8);
	TalonSRX armRight = new TalonSRX(2);
	TalonSRX climber = new TalonSRX(9);
	TalonSRX intake = new TalonSRX(7);

	DoubleSolenoid big = new DoubleSolenoid (0, 1);
	//DoubleSolenoid big2 = new DoubleSolenoid (7,8);

	Encoder enc1 = new Encoder(0, 1, true);
	Encoder enc2 = new Encoder (2, 3, false);

	ADXRS450_Gyro gyro = new ADXRS450_Gyro();
	double angle = gyro.getAngle();
	double rate = gyro.getAngle();

	Joystick left = new Joystick(0);
	Joystick right = new Joystick(2);
	Joystick controller = new Joystick(1);
	double YValue = controller.getRawAxis(1);

	JoystickButton climb = new JoystickButton (controller, 5);
	JoystickButton climbLift = new JoystickButton (right, 1);
	JoystickButton intakeForward = new JoystickButton(controller, 9);
	JoystickButton intakeReverse = new JoystickButton(controller, 10);
	//double value = armStick.getRawAxis(1);
	JoystickButton armlift = new JoystickButton(controller, 1);
	JoystickButton solenoidButton = new JoystickButton (controller, 3);

	int Distanceinticks = (t1.getSensorCollection().getQuadraturePosition());
	int Distanceinrotations = (t1.getSensorCollection().getQuadraturePosition()/4096);

	int Distanceinticks2 = (t3.getSensorCollection().getQuadraturePosition());
	int Distanceinrotationst3 = (t3.getSensorCollection().getQuadraturePosition()/4096);
	double Distanceininches = (t1.getSensorCollection().getQuadraturePosition());
	double Distanceininchest3 = (t3.getSensorCollection().getQuadraturePosition()*18.85/-4096);
	int leftmotorV = t1.getSensorCollection().getPulseWidthVelocity();
	int rightmotorV = t3.getSensorCollection().getPulseWidthVelocity();
	int DistanceinRotations = (-t1.getSensorCollection(). getQuadratureVelocity())/4096;

	double currentEncoder1Value = t1.getSensorCollection().getQuadratureVelocity();

	int encoder1Sum = 0;
	int encoder1Last = 0;      // Encoder value from the last tick
	int encoder3Sum = 0;
	int encoder3Last = 0;

	Compressor c = new Compressor(0);

	int encoder1SumGodAuton2 = 0;
	int encoder1LastGodAuton2 = 0;      // Encoder value from the last tick for god auton 2
	int encoder3SumGodAuton2 = 0;
	int encoder3LastGodAuton2 = 0;

	int encoder1SumGodAuton3 = 0;
	int encoder1LastGodAuton3 = 0;      // Encoder value from the last tick for god auton 3
	int encoder3SumGodAuton3 = 0;
	int encoder3LastGodAuton3 = 0;

	int armLeftDistanceSum = 0;
	double armLeftDistanceLast = 0;
	int armRightDistanceSum = 0;
	double armRightDistanceLast = 0;

	int SampleTime = 1000; // 1sec
	double outMin, outMax;
	boolean inAuto = false;

	int P, I, D = 1;
	int integral, previous_error, setpoint = 0;

	double kP;
	double kI;
	double kD;
	double kF;

	double pTerm;
	double iTerm;
	double dTerm;
	double fTerm;
	double CompressorValue = c.getCompressorCurrent();

	boolean running = true;
	boolean runningBehind = true;
	boolean runningStraight = true;
	boolean runningLeftRight = true;
	boolean scaleReached = false;
	boolean exchangeReached = false;

	// most of these are for god auton
	boolean partOneComplete = false;
	boolean partTwoComplete = false;
	boolean partTwoMain = false;
	boolean partThreeComplete = false;
	boolean partFourComplete = false;
	boolean partFiveComplete = false;
	boolean halfwayDone = false;
	boolean turnComplete = false;
	boolean partTwo = false;
	boolean newBlock = false;
	boolean running2 = true;
	boolean BlockPickUp = false;
	boolean running3 = true;
	boolean SwitchDrop = false;
	boolean armGoUp = false;
	boolean goTo40000 = true;

	int DIRECT = 0;
	double rcw;


	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		m_chooser.addDefault("Default Auto", kDefaultAuto);
		m_chooser.addObject("My Auto", kCustomAuto);
		SmartDashboard.putData("Auto choices", m_chooser);


		t1.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
		t3.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
		t1.setSelectedSensorPosition(0, 0, 0);
		t3.setSelectedSensorPosition(0, 0, 0);

		//dashboard = HalDashboard.getInstance();
		//this.instanceName = instanceName;
		this.kP = Math.abs(kP);
		this.kI = Math.abs(kI);
		this.kD = Math.abs(kD);
		this.kF = Math.abs(kF);
	}


	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * <p>You can add additional auto modes by adding additional comparisons to
	 * the switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	protected void initDefaultCommand() {

	}

	@Override

	public void autonomousInit() {
		m_autoSelected = m_chooser.getSelected();
		/*autoSelected = SmartDashboard.getString("Auto Selector", 
		defaultAuto);*/
		System.out.println("Auto selected: " + m_autoSelected);
		t1.getSensorCollection().setQuadraturePosition(0, 10);
		t3.getSensorCollection().setQuadraturePosition(0, 10);
		t1.getSensorCollection().setPulseWidthPosition(0, 10);
		t3.getSensorCollection().setPulseWidthPosition(0, 10);
		t1.set(ControlMode.PercentOutput, 0);
		t3.set(ControlMode.PercentOutput, 0);
		t1.setSelectedSensorPosition(0, 0, 0);
		t3.setSelectedSensorPosition(0, 0, 0);

		int encoder1Value = -t1.getSensorCollection().getQuadratureVelocity();
		int encoder3Value = t3.getSensorCollection().getQuadratureVelocity();
		encoder1Sum += encoder1Value - encoder1Last;
		encoder1Sum += encoder1Value;
		encoder3Sum += encoder3Value - encoder3Last;
		encoder3Sum += encoder3Value;

		System.out.println("Vel left: " + encoder1Value + "  last left: " + encoder1Last + "  encoder1Sum: " + encoder1Sum);
		System.out.println("Vel right: " + encoder3Value + "  last right: " + encoder3Last + "  encoder3Sum: " + encoder3Sum);
		System.out.println("Ratio: " + (encoder1Sum / (double) encoder3Sum));
		System.out.println("Position: " + -t1.getSelectedSensorPosition(0));

		encoder1Last = encoder1Value;
		encoder3Last = encoder3Value;
	}

	public void setSetpoint (int setpoint) {
		this.setpoint = setpoint;
	}

	public void armPID() {
		double error = 1; /*setpoint -mag encoder current position, switch to amt soon*/
		this.integral += (error *.2);
		double derivitave = (error - this.previous_error) / .02;
		this.rcw = P*error + I*this.integral + D*derivitave; 
	}
	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		switch (m_autoSelected) {
		case kCustomAuto:
			// Put custom auto code here
			break;
		case kDefaultAuto:
		default:
			GodAuton();

			//PUT ALL DRIVE CODE IN PERIODIC getSensorCollection().getPulseWidthPosition()
		}
	}

	// If getPulseWidthPosition() = your whatever distance wanted, do another action by setting the drive to turn to an encoder position

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopInit() {
		t1.setSelectedSensorPosition(0, 0, 0);
		t3.setSelectedSensorPosition(0, 0, 0);
		//t1.set(ControlMode.PercentOutput, 0);
		//t3.set(ControlMode.PercentOutput, 0); 
	}
	
	public void teleopPeriodic() {
		double angle = gyro.getAngle();
		tankDrive(left.getY(), right.getY()); 

		if (angle >= 360) {
			gyro.reset();
		}

		if (angle <= -360) {
			gyro.reset(); 
		}

		arm2();
		int encoder1Position= t1.getSensorCollection().getQuadratureVelocity();
		int encoderGoal = t1.getSelectedSensorPosition(21729);
		int error = encoderGoal - encoder1Position;


		kP = 0;
		kI = 0.1;
		kD = 0;

		System.out.println(angle);
		System.out.println("Position: " + t1.getSelectedSensorPosition(0));
		System.out.println("Error: " + error);
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {

	}
	public void tankDrive(double left, double right){
		t1.set(ControlMode.PercentOutput, right);
		t0.set(ControlMode.Follower, 1);
		t2.set(ControlMode.PercentOutput, -left);
		t3.set(ControlMode.Follower, 3);
	}

	void driveStraight() {
		int encoder1Value = -t1.getSensorCollection().getQuadratureVelocity();
		int encoder3Value = t3.getSensorCollection().getQuadratureVelocity();
		encoder1Sum += encoder1Value - encoder1Last;
		encoder1Sum += encoder1Value;
		encoder3Sum += encoder3Value - encoder3Last;
		encoder3Sum += encoder3Value;

		System.out.println("Vel left: " + encoder1Value + "  last left: " + encoder1Last + "  encoder1Sum: " + encoder1Sum);
		System.out.println("Vel right: " + encoder3Value + "  last right: " + encoder3Last + "  encoder3Sum: " + encoder3Sum);
		System.out.println("Ratio: " + (encoder1Sum / (double) encoder3Sum));

		encoder1Last = encoder1Value;
		encoder3Last = encoder3Value;
		//t1.configMotionAcceleration(400, 10); //400 per second (DONT USE!!!!!!)
		//t1.configMotionCruiseVelocity(400, 10); //drive at 400 (DONT USE!!!!!), just a test	4`
	}

	public void arm2(){
		/*t2.config_kD(0, 0.1, 10);
		t2.config_kI(0, 0.1, 10);
		t2.config_kF(0, 0.1, 10);
		t2.config_kP(0, 0.1, 10);

		t1.config_kD(0, 0.1, 10);
		t1.config_kI(0, 0.1, 10);
		t1.config_kF(0, 0.1, 10);
		t1.config_kP(0, 0.1, 10);


		t2.configMotionAcceleration(1, 10);
		t2.configMotionCruiseVelocity(1, 10);
		t1.configMotionAcceleration(1, 10);
		t1.configMotionCruiseVelocity(1, 10);

		t2.set(ControlMode.MotionMagic, 10);
		t1.set(ControlMode.MotionMagic, 10);*/

		if(armlift.get()){
			armGoUp = true;
		}else if(!armlift.get()) {
			armGoUp = false;
		}

		if(armGoUp == true) {
			System.out.println("ARM GO UP: " + armGoUp );	
			//armLeft.set(ControlMode.PercentOutput, 0.03);
			//armRight.set(ControlMode.PercentOutput, -0.03);
		}else if(armGoUp == false) {
			System.out.println("ARM GO UP: " + armGoUp);
			//armLeft.set(ControlMode.PercentOutput, 0);
			//armRight.set(ControlMode.PercentOutput, 0);
		}
	}

	void arm() {
		t1.configMotionAcceleration(1, 10);
		t1.configMotionCruiseVelocity(1, 10);

		double armLeftDistance = enc1.getDistance();
		double armRightDistance = enc2.getDistance();
		double angle = gyro.getAngle();
		double rate = gyro.getAngle();
		armLeftDistanceSum += armLeftDistance - armLeftDistanceLast;
		armLeftDistanceSum += armLeftDistance;
		armRightDistanceSum += armRightDistance - armRightDistanceLast;
		armRightDistanceSum += armRightDistance;;

		enc1.setDistancePerPulse(.1885);

		armLeftDistanceLast = armLeftDistance;
		armRightDistanceLast = armRightDistance;

		if(armlift.get()) {
			if (armLeftDistanceSum > armRightDistanceSum) {
				armLeft.set(ControlMode.PercentOutput,0.03);
				armRight.set(ControlMode.PercentOutput, -0.05);
				//System.out.print
			}else if (armLeftDistanceSum < armRightDistanceSum) {
				armLeft.set(ControlMode.PercentOutput, 0.05);
				armRight.set(ControlMode.PercentOutput, -0.03);
			}else{
				armLeft.set(ControlMode.PercentOutput, 0.03);
				armRight.set(ControlMode.PercentOutput, -0.03);
			}

		}else{
			armLeft.set(ControlMode.PercentOutput, 0);
			armRight.set(ControlMode.PercentOutput, 0);
		}
	}

	void auton6(){
		//gyro.reset();

		double angle = gyro.getAngle();
		double rate = gyro.getRate();

		int encoder1Value = -t1.getSensorCollection().getQuadratureVelocity();
		int encoder3Value = t3.getSensorCollection().getQuadratureVelocity();

		//int encoderRatio = encoder1Sum/encoder3Sum;
		encoder1Sum += encoder1Value - encoder1Last;
		encoder1Sum += encoder1Value;
		encoder3Sum += encoder3Value - encoder3Last;
		encoder3Sum += encoder3Value;

		System.out.println("Vel left: " + encoder1Value + "  last left: " + encoder1Last + "  encoder1Sum: " + encoder1Sum);
		System.out.println("Vel right: " + encoder3Value + "  last right: " + encoder3Last + "  encoder3Sum: " + encoder3Sum);
		System.out.println("Angle: " + angle);
		System.out.println("Position: " + t1.getSelectedSensorPosition(0));
		encoder1Last = encoder1Value;
		encoder3Last = encoder3Value;

		if(encoder1Sum > encoder3Sum && t1.getSelectedSensorPosition(0) < 500) {
			tankDrive(.24,.27);
			if(-t1.getSelectedSensorPosition(0) > 20000 && angle < 45){
				System.out.println("Angle: " + angle);
				tankDrive(0,0);
				Timer.delay(2);
				System.out.println("Angle: " + angle);
				tankDrive(.35,.0);
				Timer.delay(1); 
				tankDrive(0,0);
			}
		}else if(encoder1Sum < encoder3Sum && t1.getSelectedSensorPosition(0) < 500) {
			tankDrive(.27, .24);
		}else if (encoder1Sum == encoder3Sum){
			tankDrive(.25,.25);
		}else{
			tankDrive(0,0);

			//217.29 ticks per inch
			//2,607.48 ticks per feet
		}
	}

	void autonStraightLine() {
		int encoder1Value = -t1.getSensorCollection().getQuadratureVelocity();
		int encoder3Value = t3.getSensorCollection().getQuadratureVelocity();
		encoder1Sum += encoder1Value - encoder1Last;
		encoder1Sum += encoder1Value;
		encoder3Sum += encoder3Value - encoder3Last;
		encoder3Sum += encoder3Value;

		double angle = gyro.getAngle();
		double rate = gyro.getRate();

		System.out.println("Vel left: " + encoder1Value + "  last left: " + encoder1Last + "  encoder1Sum: " + encoder1Sum);
		System.out.println("Vel right: " + encoder3Value + "  last right: " + encoder3Last + "  encoder3Sum: " + encoder3Sum);
		System.out.println("Ratio: " + (encoder1Sum / (double) encoder3Sum));
		System.out.println("Position: " + -t1.getSelectedSensorPosition(0));
		System.out.println("Angle: " + angle);
		encoder1Last = encoder1Value;
		encoder3Last = encoder3Value;


		if(encoder1Sum  > encoder3Sum && running == true) {
			tankDrive(.24,.27);

		}else if(encoder1Sum < encoder3Sum && running == true) {
			tankDrive(.27, .24);
		}else if(encoder1Sum == encoder3Sum && running == true) {
			tankDrive(.25, .25);
		}else if(encoder1Sum > encoder3Sum && running == false) {
			tankDrive(0,0);
		}else if(encoder1Sum < encoder3Sum && running == false) {
			tankDrive(0,0);
		}else if(encoder1Sum == encoder3Sum && running == false) {
			tankDrive(0,0);
		}

		if (-t1.getSelectedSensorPosition(0) >= 31174) { //Crossing the auton line
			running = false;
		}
		if (running == false && angle < 85) {
			tankDrive(.35, 0);
		}else if(angle >= 85) {
			tankDrive(0,0);
			Timer.delay(1);
			tankDrive(0,0);
		}
	}

	void autonrightback(){ 
		int encoder1Value = -t1.getSensorCollection().getQuadratureVelocity();
		int encoder3Value = t3.getSensorCollection().getQuadratureVelocity();
		double angle = gyro.getAngle();
		double rate = gyro.getAngle();
		encoder1Sum += encoder1Value - encoder1Last;
		encoder1Sum += encoder1Value;
		encoder3Sum += encoder3Value - encoder3Last;
		encoder3Sum += encoder3Value;

		System.out.println("Vel left: " + encoder1Value + "  last left: " + encoder1Last + "  encoder1Sum: " + encoder1Sum);
		System.out.println("Vel right: " + encoder3Value + "  last right: " + encoder3Last + "  encoder3Sum: " + encoder3Sum);
		System.out.println("Ratio: " + (encoder1Sum / (double) encoder3Sum));
		System.out.println("Position: " + -t1.getSelectedSensorPosition(0));
		System.out.println("Angle: " + gyro.getAngle());
		System.out.println(runningStraight);
		encoder1Last = encoder1Value;
		encoder3Last = encoder3Value;

		double distanceAMT = enc1.getDistance();


		if(encoder1Sum > encoder3Sum && runningStraight == true){
			tankDrive(.24, .27);
		}else if(encoder1Sum < encoder3Sum && runningStraight == true){
			tankDrive(.27, .24);
		}else if(encoder1Sum == encoder3Sum && runningStraight == true) {
			tankDrive(.25, .25);
		}

		if (-t1.getSelectedSensorPosition(0) >= 52990.70 && angle > -80 && runningStraight == true) { //Crossing the auton line
			tankDrive(0, .35);
			Timer.delay(1);
		}

		if (-t1.getSelectedSensorPosition(0) >= 70000 && angle < -80) { //Crossing the switch line
			runningStraight = false;
		}

		if (runningStraight == false) {
			tankDrive(0,0);
		}


		if(angle >= 360) {
			gyro.reset();
		}

		if (angle < -360) {
			gyro.reset();
		}
	}

	void straightRightLeft(){
		int encoder1Value = -t1.getSensorCollection().getQuadratureVelocity();
		int encoder3Value = t3.getSensorCollection().getQuadratureVelocity();
		encoder1Sum += encoder1Value - encoder1Last;
		encoder1Sum += encoder1Value;
		encoder3Sum += encoder3Value - encoder3Last;
		encoder3Sum += encoder3Value;

		System.out.println("Vel left: " + encoder1Value + "  last left: " + encoder1Last + "  encoder1Sum: " + encoder1Sum);
		System.out.println("Vel right: " + encoder3Value + "  last right: " + encoder3Last + "  encoder3Sum: " + encoder3Sum);
		System.out.println("Ratio: " + (encoder1Sum / (double) encoder3Sum));
		System.out.println("Position: " + -t1.getSelectedSensorPosition(0));
		System.out.println("Angle: " + gyro.getAngle());
		System.out.println(runningStraight);
		encoder1Last = encoder1Value;
		encoder3Last = encoder3Value;

		double angle = gyro.getAngle();
		runningStraight = true;

		if(encoder1Sum > encoder3Sum && runningStraight == true){ 
			tankDrive(.24, .27);
		}else if(encoder1Sum < encoder3Sum && runningStraight == true){
			tankDrive(.27, .24);
		}else if(encoder1Sum == encoder3Sum && runningStraight == true) { 
			tankDrive(.25, .25);
		}
	}

	void scaleAuton() {
		int encoder1Value = -t1.getSensorCollection().getQuadratureVelocity();
		int encoder3Value = t3.getSensorCollection().getQuadratureVelocity();
		encoder1Sum += encoder1Value - encoder1Last;
		encoder1Sum += encoder1Value;
		encoder3Sum += encoder3Value - encoder3Last;
		encoder3Sum += encoder3Value;

		System.out.println("Vel left: " + encoder1Value + "  last left: " + encoder1Last + "  encoder1Sum: " + encoder1Sum);
		System.out.println("Vel right: " + encoder3Value + "  last right: " + encoder3Last + "  encoder3Sum: " + encoder3Sum);
		System.out.println("Ratio: " + (encoder1Sum / (double) encoder3Sum));
		System.out.println("Position: " + -t1.getSelectedSensorPosition(0));
		System.out.println("Angle: " + gyro.getAngle());
		encoder1Last = encoder1Value;
		encoder3Last = encoder3Value;

		double angle = gyro.getAngle();
		scaleReached = false;

		if(encoder1Sum > encoder3Sum && scaleReached == false){ 
			tankDrive(.24, .27);
		}else if(encoder1Sum < encoder3Sum && scaleReached == false) {
			tankDrive(.27, .24);
		}else if(encoder1Sum == encoder3Sum && scaleReached == false) {
			tankDrive(.25, .25);
		}

		if(-t1.getSelectedSensorPosition(0) >= 30000 && scaleReached == false) {
			scaleReached = true;
		}

		if (-t1.getSelectedSensorPosition(0) >= 30000 && angle < 75  && scaleReached == true) {
			tankDrive(.35, 0);
		}else if(angle >= 75 && scaleReached == true) {
			tankDrive(-.25, -.25);  
			Timer.delay(2);
			tankDrive(.25, .25);
			Timer.delay(2);
			tankDrive(0, 0);
			Timer.delay(5); 
		}
	} 

	void solenoidVoid() {
		if(solenoidButton.get()) {
			big.set(Value.kForward);
		}else{
			big.set(Value.kReverse);
		}
	}

	void calibrate() {
		gyro.calibrate();
	}

	void autonLeftRight(){

		int encoder1Value = -t1.getSensorCollection().getQuadratureVelocity();
		int encoder3Value = t3.getSensorCollection().getQuadratureVelocity();
		encoder1Sum += encoder1Value - encoder1Last;
		encoder1Sum += encoder1Value;
		encoder3Sum += encoder3Value - encoder3Last;
		encoder3Sum += encoder3Value;

		double angle = gyro.getAngle();
		double rate = gyro.getRate();


		System.out.println("Vel left: " + encoder1Value + "  last left: " + encoder1Last + "  encoder1Sum: " + encoder1Sum);
		System.out.println("Vel right: " + encoder3Value + "  last right: " + encoder3Last + "  encoder3Sum: " + encoder3Sum);
		System.out.println("Ratio: " + (encoder1Sum / (double) encoder3Sum));
		System.out.println("Position: " + -t1.getSelectedSensorPosition(0));
		System.out.println("Angle: " + gyro.getAngle());
		encoder1Last = encoder1Value;
		encoder3Last = encoder3Value;

		if(encoder1Sum > encoder3Sum){ 
			tankDrive(.24, .27);
		}else if(encoder1Sum < encoder3Sum){
			tankDrive(.27, .24);
		}else if(encoder1Sum == encoder3Sum) {
			tankDrive(.25, .25);
		}

		if (-t1.getSelectedSensorPosition(0) >= 15251.896) { //Crossing the switch line
			runningLeftRight = false;
		}

		if (runningLeftRight == false && angle < 85) {
			tankDrive(.35, 0);
		}else if(angle >= 85){
			autonStraightLine();
		}
	}

	void autonExchange(){

		double angle = gyro.getAngle();

		int encoder1Value = -t1.getSensorCollection().getQuadratureVelocity();
		int encoder3Value = t3.getSensorCollection().getQuadratureVelocity();
		encoder1Sum += encoder1Value - encoder1Last;
		encoder1Sum += encoder1Value;
		encoder3Sum += encoder3Value - encoder3Last;
		encoder3Sum += encoder3Value;

		System.out.println("Vel left: " + encoder1Value + "  last left: " + encoder1Last + "  encoder1Sum: " + encoder1Sum);
		System.out.println("Vel right: " + encoder3Value + "  last right: " + encoder3Last + "  encoder3Sum: " + encoder3Sum);
		System.out.println("Ratio: " + (encoder1Sum / (double) encoder3Sum));
		System.out.println("Position: " + -t1.getSelectedSensorPosition(0));
		System.out.println("Exchange Reached: " + exchangeReached);
		System.out.println("Angle: " + gyro.getAngle());
		System.out.println("Half Done: " + halfwayDone);
		System.out.println("Exchange Reached: " + exchangeReached);
		System.out.println("Turn Complete: " + turnComplete);
		System.out.println("New Block: " + newBlock);


		encoder1Last = encoder1Value;
		encoder3Last = encoder3Value;


		if (encoder1Sum > encoder3Sum && exchangeReached == false && halfwayDone == false && newBlock == false) {
			tankDrive(.24, .27);
		}else if(encoder1Sum < encoder3Sum && exchangeReached == false && halfwayDone == false && newBlock == false){
			tankDrive(.27, .24);
		}else if(encoder1Sum == encoder3Sum && exchangeReached == false && halfwayDone == false && newBlock == false ){
			tankDrive(.25, .25);
		}

		if(-t1.getSelectedSensorPosition(0) >= 15251.896 && exchangeReached == false){
			exchangeReached = true; 
		}


		if(-t1.getSelectedSensorPosition(0) >= 20000 && angle < 75 && exchangeReached == true && halfwayDone == false && newBlock == false){
			tankDrive(.35, 0);
		}else if(angle >= 75 && exchangeReached == true && halfwayDone == false){
			tankDrive(-.25, -.25);
			//intake.set(ControlMode.PercentOutput, -1); (add when we use the actual robot cause the practice one is dummy stupid)
			Timer.delay(2);
			tankDrive(.25, .25);
			Timer.delay(1);
			halfwayDone = true;
		}

		if(halfwayDone == true && angle < 170 && -t1.getSelectedSensorPosition(0) >= 20000 && turnComplete == false) {
			tankDrive(.35, 0);
		}else if(halfwayDone == true && angle >= 170 && -t1.getSelectedSensorPosition(0) < 50000 && turnComplete == false) {
			tankDrive(.25, .25);
			partTwo = true;
			turnComplete = true;
		}

		if(gyro.getAngle() > 360) {
			gyro.reset(); 
		}else if (gyro.getAngle() < -360) {
			gyro.reset();
		}

		if(partTwo = true && angle <= 250 && newBlock == false && exchangeReached == true && turnComplete == true && -t1.getSelectedSensorPosition(0) >= 40000) {
			tankDrive(.35, 0);
			Timer.delay(1);
		}else if(partTwo = true && angle >= 250 && newBlock == false && exchangeReached == true && turnComplete == true) {
			tankDrive(.10, .10);
			Timer.delay(2);
			//insert intake code here (picking up a new block)
			tankDrive(-.10, -.10);
			tankDrive(0, -.35);
			newBlock = true; 

		}else if (partTwo = true && angle > 250 && newBlock == true && exchangeReached == true && turnComplete == true){
			tankDrive(0, 0);
			//Inset angle of portal here
		}
	}

	void GodAuton() { //This auton code will out a preset block on the switch, grab one from the back of the switch, and carry on to the scale where they will add another block, go back to the start, pick up a block and put it in the exchange
		//Part 1
		int encoder1Value = -t1.getSensorCollection().getQuadratureVelocity();
		int encoder3Value = t3.getSensorCollection().getQuadratureVelocity();
		encoder1Sum += encoder1Value - encoder1Last;
		encoder1Sum += encoder1Value;
		encoder3Sum += encoder3Value - encoder3Last;
		encoder3Sum += encoder3Value;

		double angle = gyro.getAngle();
		double rate = gyro.getRate();

		System.out.println("Vel left: " + encoder1Value + "  last left: " + encoder1Last + "  encoder1Sum: " + encoder1Sum);
		System.out.println("Vel right: " + encoder3Value + "  last right: " + encoder3Last + "  encoder3Sum: " + encoder3Sum);
		System.out.println("Ratio: " + (encoder1Sum / (double) encoder3Sum));
		System.out.println("Position: " + -t1.getSelectedSensorPosition(0));
		System.out.println("Angle: " + angle);
		System.out.println("Part Two Main: "  + partTwoMain);
		encoder1Last = encoder1Value;
		encoder3Last = encoder3Value;


		if(encoder1Sum  > encoder3Sum && running == true && partOneComplete == false && partTwoComplete == false && partThreeComplete == false) {
			tankDrive(.24,.27);
		}else if(encoder1Sum < encoder3Sum && running == true && partOneComplete == false && partTwoComplete == false && partThreeComplete == false) {
			tankDrive(.27, .24);
		}else if(encoder1Sum == encoder3Sum && running == true && partOneComplete == false && partTwoComplete == false && partThreeComplete == false) {
			tankDrive(.25, .25);
		}


		if (-t1.getSelectedSensorPosition(0) >= 31174 && partOneComplete == false) { //Crossing the auton line
			running = false;
		}

		if (running == false && angle < 85 && partOneComplete == false && SwitchDrop == false && partOneComplete == false && partTwoComplete == false && partThreeComplete == false) {
			tankDrive(.35, 0);
		}else if(angle >= 85 && partOneComplete == false && running == false && SwitchDrop == false && partOneComplete == false && partTwoComplete == false && partThreeComplete == false) {
			tankDrive(.10,.10);
			//insert arm code here, raising, shooting than lowering
			Timer.delay(1);
			tankDrive(-.10, -.10);
			Timer.delay(1);
			SwitchDrop = true;
		} 

		if(partOneComplete == false && angle >= 70 && partOneComplete == false && partTwoComplete == false && partThreeComplete == false&& SwitchDrop == true) {
			tankDrive(-.35, 0);
		}else if(partOneComplete == false && angle < 15 && partOneComplete == false && partTwoComplete == false && partThreeComplete == false && SwitchDrop == true) {
			partOneComplete = true;
		}

		if(encoder1Sum  > encoder3Sum && running2 == true && partOneComplete == true && partTwoComplete == false && partThreeComplete == false && SwitchDrop == true) {
			tankDrive(.24,.27);
		}else if(encoder1Sum < encoder3Sum && running2 == true && partOneComplete == true && partTwoComplete == false && partThreeComplete == false && SwitchDrop == true) {
			tankDrive(.27, .24);
		}else if(encoder1Sum == encoder3Sum && running2 == true && partOneComplete == true && partTwoComplete == false && partThreeComplete == false && SwitchDrop == true) {
			tankDrive(.25, .25);
		}  


		if (-t1.getSelectedSensorPosition(0) >= 60000 && partTwoComplete == false  && running2 == true && partOneComplete == true && partThreeComplete == false && SwitchDrop == true) {
			running2 = false;
		} 

		if(running2 == false && angle < 45 && -t1.getSelectedSensorPosition(0) >= 60000 && partTwoComplete == false && BlockPickUp == false && partOneComplete == true && partThreeComplete == false && SwitchDrop == true) {
			tankDrive(.35, 0);
		}else if(angle >= 75 && -t1.getSelectedSensorPosition(0) > 60000 && partTwoComplete == false && running2 == false && BlockPickUp == false && partOneComplete == true && partThreeComplete == false && SwitchDrop == true) {
			tankDrive(.10, .10);
			Timer.delay(2);
			intake.set(ControlMode.PercentOutput, 1);
			Timer.delay(1);
			intake.set(ControlMode.PercentOutput, 0);
			tankDrive(-.10, -.10);
			Timer.delay(2); 
			tankDrive(-.35,0);  
			BlockPickUp = true;
		}

		if(partTwoComplete == false && running2 == false && SwitchDrop == true && angle < 10 && BlockPickUp == true && partThreeComplete == false && partOneComplete == true) {
			partTwoComplete = true;		
		}  


		if(encoder1Sum > encoder3Sum && partThreeComplete == false && running3 == true && -t1.getSelectedSensorPosition(0) < 90000 && angle < 20 && running3 == true && partTwoComplete == true && partOneComplete == true && SwitchDrop == true && BlockPickUp == true){
			tankDrive(.24, .27);
		}else if(encoder1Sum < encoder3Sum && partThreeComplete == false && running3 == true && -t1.getSelectedSensorPosition(0) < 90000 && angle < 20 && running3 == true && partTwoComplete == true && partOneComplete == true && SwitchDrop == true && BlockPickUp == true){
			tankDrive(.27, .24); 
		}else if(encoder1Sum == encoder3Sum && partThreeComplete == false && running3 == true && -t1.getSelectedSensorPosition(0) < 90000 && angle < 20 && running == true && partTwoComplete == true && partOneComplete == true && SwitchDrop == true && BlockPickUp == true){
			tankDrive(.25, .25);
		}

		if(angle < 20 && -t1.getSelectedSensorPosition(0) >= 90000 && partThreeComplete == false && running3 == true && partTwoComplete == true && partOneComplete == true && SwitchDrop == true && BlockPickUp == true) { 
			running3 = false;
		}

		if(angle < 20 && running3 == false && partThreeComplete == false && partTwoComplete == true && partOneComplete == true && SwitchDrop == true && BlockPickUp == true) {
			tankDrive(.35, 0);
		}else if(angle >= 75 && -t1.getSelectedSensorPosition(0) > 90000 && partThreeComplete == false && partTwoComplete == true && partOneComplete == true && running3 == false && SwitchDrop == true && BlockPickUp == true) {
			tankDrive(-.25, -.25);
			Timer.delay(2);
			tankDrive(.10, .10);
			//insert arm code here
			Timer.delay(2);
			tankDrive(-.25, -.25);
			Timer.delay(1);
		}
	}

	void GodAuton2() {
		//Part 2
		int encoder1ValueGodAuton2 = -t1.getSensorCollection().getQuadratureVelocity();
		int encoder3ValueGodAuton2 = t3.getSensorCollection().getQuadratureVelocity();
		encoder1SumGodAuton2 += encoder1ValueGodAuton2 - encoder1LastGodAuton2;
		encoder1SumGodAuton2 += encoder1ValueGodAuton2;
		encoder3SumGodAuton2 += encoder3ValueGodAuton2 - encoder3LastGodAuton2;
		encoder3SumGodAuton2 += encoder3ValueGodAuton2;

		double angle = gyro.getAngle();
		double rate = gyro.getRate(); 

		System.out.println("Vel left: " + encoder1ValueGodAuton2 + "  last left: " + encoder1LastGodAuton2 + "  encoder1SumGodAuton2: " + encoder1SumGodAuton2);
		System.out.println("Vel right: " + encoder3ValueGodAuton2 + "  last right: " + encoder3LastGodAuton2 + "  encoder3SumGodAuton2: " + encoder3SumGodAuton2);
		System.out.println("Ratio: " + (encoder1SumGodAuton2 / (double) encoder3SumGodAuton2));
		System.out.println("Position: " + -t1.getSelectedSensorPosition(0));
		System.out.println("Angle: " + angle);
		System.out.println("Part One Complete: "  + partOneComplete);
		System.out.println("Part 2 Complete: "  + partTwoComplete);
		System.out.println("Part Two Main: "  + partTwoMain);
		System.out.println("Running: " + running2);
		System.out.println("Block Pick Up: " + BlockPickUp);

		encoder1LastGodAuton2 = encoder1ValueGodAuton2;
		encoder3LastGodAuton2 = encoder3ValueGodAuton2;

		if(encoder1SumGodAuton2  > encoder3SumGodAuton2 && partTwoComplete == false && running2 == true) {
			tankDrive(.24,.27);
		}else if(encoder1SumGodAuton2 < encoder3SumGodAuton2 && partTwoComplete == false && running2 == true) {
			tankDrive(.27, .24);
		}else if(encoder1SumGodAuton2 == encoder3SumGodAuton2 && partTwoComplete == false && running2 == true) {
			tankDrive(.25, .25);
		}  


		if (-t1.getSelectedSensorPosition(0) >= 60000 && partTwoComplete == false  && running2 == true) {
			running2 = false;
		} 

		if(running2 == false && angle < 45 && -t1.getSelectedSensorPosition(0) >= 60000 && partTwoComplete == false && BlockPickUp == false && partThreeComplete == false) {
			tankDrive(.35, 0);
		}else if(angle >= 75 && -t1.getSelectedSensorPosition(0) > 60000 && partTwoComplete == false && running2 == false && BlockPickUp == false && partThreeComplete == false) {
			tankDrive(.10, .10);
			Timer.delay(2);
			intake.set(ControlMode.PercentOutput, 1);
			Timer.delay(1);
			intake.set(ControlMode.PercentOutput, 0);
			tankDrive(-.10, -.10);
			Timer.delay(2); 
			tankDrive(-.35,0);  
			BlockPickUp = true;
		}
		if(partTwoComplete == false && running2 == false && angle < 10 && BlockPickUp == true && partThreeComplete == false) {
			partTwoComplete = true;		
		}  
	}

	void GodCode3() { //This also does the same thing as God auton 1, but a different part
		//Part 3

		int encoder1ValueGodAuton3 = -t1.getSensorCollection().getQuadratureVelocity();
		int encoder3ValueGodAuton3 = t3.getSensorCollection().getQuadratureVelocity();
		encoder1SumGodAuton3 += encoder1ValueGodAuton3 - encoder1LastGodAuton3;
		encoder1SumGodAuton3 += encoder1ValueGodAuton3;
		encoder3SumGodAuton3 += encoder3ValueGodAuton3 - encoder3LastGodAuton3;
		encoder3SumGodAuton3 += encoder3ValueGodAuton3;

		double angle = gyro.getAngle();
		double rate = gyro.getRate();

		System.out.println("Vel left: " + encoder1ValueGodAuton3 + "  last left: " + encoder1LastGodAuton3 + "  encoder1SumGodAuton3: " + encoder1SumGodAuton3);
		System.out.println("Vel right: " + encoder3ValueGodAuton3 + "  last right: " + encoder3LastGodAuton3 + "  encoder3SumGodAuton3: " + encoder3SumGodAuton3);
		System.out.println("Ratio: " + (encoder1SumGodAuton3 / (double) encoder3SumGodAuton3));
		System.out.println("Position: " + -t1.getSelectedSensorPosition(0));
		System.out.println("Angle: " + angle);
		System.out.println("Part Three Complete: " + partThreeComplete);
		encoder1LastGodAuton3 = encoder1ValueGodAuton3;
		encoder3LastGodAuton3 = encoder3ValueGodAuton3;

		partOneComplete = true;
		partTwoComplete = true;

		if(encoder1SumGodAuton3 > encoder3SumGodAuton3 && partThreeComplete == false && running3 == true && -t1.getSelectedSensorPosition(0) < 90000 && angle < 20 && running3 == true && partTwoComplete == true){
			tankDrive(.24, .27);
		}else if(encoder1SumGodAuton3 < encoder3SumGodAuton3 && partThreeComplete == false && running3 == true && -t1.getSelectedSensorPosition(0) < 90000 && angle < 20 && running3 == true && partTwoComplete == true){
			tankDrive(.27, .24); 
		}else if(encoder1SumGodAuton3 == encoder3SumGodAuton3 && partThreeComplete == false && running3 == true && -t1.getSelectedSensorPosition(0) < 90000 && angle < 20 && running == true && partTwoComplete == true){
			tankDrive(.25, .25);
		}

		if(angle < 20 && -t1.getSelectedSensorPosition(0) >= 90000 && partThreeComplete == false && running3 == true) {
			running3 = false;
		}

		if(angle < 20 && running3 == false && partThreeComplete == false) {
			tankDrive(.35, 0);
		}else if(angle >= 75 && -t1.getSelectedSensorPosition(0) > 90000 && partThreeComplete == false && running3 == false) {
			tankDrive(-.25, -.25);
			Timer.delay(2);
			tankDrive(.10, .10);
			//insert arm code here
			Timer.delay(2);
			tankDrive(-.25, -.25);
			Timer.delay(1);
		}
	}

	void CalcWheelRotations() {
		System.out.println("Distance In Rotations: " + DistanceinRotations);
		tankDrive(.5, .5);
		Timer.delay(60);
	}
}