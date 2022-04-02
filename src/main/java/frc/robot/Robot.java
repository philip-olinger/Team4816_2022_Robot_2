// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
// import edu.wpi.first.wpilibj.Joystick;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxPIDController;
// import com.revrobotics.REVLibError;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType.*;
import com.revrobotics.SparkMaxRelativeEncoder;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * Declare all your objects here
   *
   * MotorControllers (CANSparkMax)
   * MotorControllerGroup
   * DifferentialDrive
   * Joystick or XboxController (or other controller types)
   * Timer (to measure elapsed time in Autonomous mode)
   */
  private CANSparkMax m_leftFrontMotor, m_leftRearMotor; 
  private CANSparkMax m_rightFrontMotor, m_rightRearMotor;
  private CANSparkMax m_intakeMotor;
  private CANSparkMax m_winchMotorA, m_winchMotorB;
  private MotorControllerGroup m_leftMotors, m_rightMotors;
  private DifferentialDrive m_robotDrive;
  private boolean m_intake_on;  // Keep track of intake toggled on/off
  private XboxController m_stick;
  private XboxController n_stick;
  private final Timer m_timer = new Timer();
  // Experimental PID control parameters
  private SparkMaxRelativeEncoder m_encoder;
  private SparkMaxPIDController m_PIDController;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    /**
     * Instantiate your objects from above and assign them here
     * 
     * CAN Bus device IDS: 
     * https://docs.wpilib.org/en/latest/docs/software/can-devices/can-addressing.html
     * 
     * CANSparkMax API documentation
     * https://codedocs.revrobotics.com/java/com/revrobotics/package-summary.html
     */
    // Instantiate all motors
    m_rightFrontMotor = new CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless);
    m_rightRearMotor = new CANSparkMax(6, CANSparkMaxLowLevel.MotorType.kBrushless);
    m_leftFrontMotor = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless);
    m_leftRearMotor = new CANSparkMax(7, CANSparkMaxLowLevel.MotorType.kBrushless);
    m_intakeMotor = new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless);    
    m_winchMotorA = new CANSparkMax(5, CANSparkMaxLowLevel.MotorType.kBrushless);
    m_winchMotorB = new CANSparkMax(8, CANSparkMaxLowLevel.MotorType.kBrushless);
    // Instantiate joysticks
    m_stick = new XboxController(0);
    n_stick = new XboxController(1);

    /**
    * CONFIGURE THE DRIVE MOTORS AND DRIVETRAIN
    */
    // For 2022, we may need to set one side's drive motors to inverted
    m_rightFrontMotor.setInverted(false);
    m_rightRearMotor.setInverted(false);
    m_leftFrontMotor.setInverted(true);
    m_leftRearMotor.setInverted(true);

    m_leftMotors = new MotorControllerGroup(m_leftFrontMotor, m_leftRearMotor);
    m_rightMotors = new MotorControllerGroup(m_rightFrontMotor, m_rightRearMotor);
    m_robotDrive = new DifferentialDrive(m_leftMotors, m_rightMotors);

    /**
     * CONFIGURE THE INTAKE MOTOR
     */
    m_intakeMotor.setInverted(false);
    // Set CANSparkMax idle modes if needed (kCoast or kBrake)
    m_intakeMotor.setIdleMode(IdleMode.kCoast); // Maybe this will solve the single click?
    m_intake_on = false;  // Intake motor on/off toggle variable (used later)

    /**
     * CONFIGURE THE WINCH MOTORS
     */
    /**
     *  Set one winch motor to follow the other, but since they are facing opposite
     *  directions make it follow with inverted=true
     */
    m_winchMotorB.follow(m_winchMotorA, true);   
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    /* Set up the Timer for measuring how long we drive in autonomous */
    m_timer.reset();
    m_timer.start();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // Drive for 2 seconds
    if (m_timer.get() < 3.0) {
      /**
      * Operate the robotDrive subclass to drive the robot
      */
      m_robotDrive.arcadeDrive(-0.5, 0.0); // drive forwards half speed

    } else {
      m_robotDrive.stopMotor(); // stop robot
    }
    //break; // Exit the case statement
    //}
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // For arcade drive, we use just the left stick
    // m_robotDrive.arcadeDrive(m_stick.getLeftY(), m_stick.getLeftX());
    
    // For tank drive, we need two sticks
    double y_val = m_stick.getLeftY();
    double x_val = m_stick.getRightY();

    // Keep the sign for later on (+ for forward, - for backward)
    double y_sign = 1.0;
    double x_sign = 1.0;
    if(y_val < 0){
      y_sign = -y_sign;
    }
    if(x_val < 0){
      x_sign = -x_sign;
    }
    // Tone down the top speed
    y_val *= 0.9;
    x_val *= 0.9;
    // Square the inputs for exponential response, but keep the sign
    m_robotDrive.tankDrive((y_sign * (y_val * y_val)),
                           (x_sign * (x_val * x_val)));

    // Control the intake by toggling on and off with right front bumper
    if(n_stick.getRightBumperPressed()){
      if(m_intake_on == false){
        // Toggle on
        m_intakeMotor.set(0.25);  // Speed is 0.25
        m_intake_on = true;
      } else {
        // Toggle off
        m_intakeMotor.stopMotor();
        m_intake_on = false;
      }
    };

    // Control the winch with right stick of 2nd XboxController
    m_winchMotorA.set(n_stick.getRightY());

    /**
     * Experimental PID Control - refer to:
     * https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/pid-video.html
     * https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/Java/Encoder%20Feedback%20Device/src/main/java/frc/robot/Robot.java
     */    
    /**
     * PID equation coefficients - we will set via Ziegler-Nichols method
     * 1. Set kI, kD to 0
     * 2. Increase Kp until steady oscillation (not growing oscillation) 
     *    This is Kc (critical gain) and Pc (period of oscillation)
     * 3. Your final values should be: Kp = 0.6*Kc ; Ki = 2*Kp/Pc ; Kd = 0.125*Kp*Pc

     * 
    kP = 0.1; // Constant for proportional input
    kI = 1e-4; // Constant for integral input
    KD = 1; // Constant for derivative input
    kIz = 0; // Constant for iZone (?))
    kFF = 0; // Constant for Feed-Forward Gain (?)
    kMinOutput = -1;
    kMaxOutput = 1; 

    m_encoder = m_winchMotorA.getEncoder(); 
    m_PIDController = m_winchMotorA.getPIDController();
    m_PIDController.setFeedbackDevice(m_encoder);
    m_PIDController.setP(kP);
    m_PIDController.setI(kI);
    m_PIDController.setD(kD);
    m_PIDController.setIZone(kIz);
    m_PIDController.setFF(kFF);
    m_PIDController.setOutputRange(kMinOutput, kMaxOutput);
    /* */
  }

  @Override
  public void teleopExit() {
    m_intakeMotor.stopMotor();
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
