// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkMaxLowLevel.MotorType.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /* Sample code to interact with smartdashboard */
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

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
  // Soon to include pickup roller motor and others
  private CANSparkMax m_intakeMotor;
  private MotorControllerGroup m_leftMotors, m_rightMotors;
  private DifferentialDrive m_robotDrive;
  private XboxController m_stick;
  private final Timer m_timer = new Timer();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    /* More smartdashboard integration */
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    /**
     * Instantiate your objects from above and assign them here
     * 
     * CAN Bus device IDS: 
     * https://docs.wpilib.org/en/latest/docs/software/can-devices/can-addressing.html
     */
    m_rightFrontMotor = new CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless);
    m_rightRearMotor = new CANSparkMax(6, CANSparkMaxLowLevel.MotorType.kBrushless);
    
    /*
    // For 2022, we may need to set the right-side motors to inverted
    m_rightFrontMotor.setInverted(true);
    m_rightRearMotor.setInverted(true);

    // We can set them to inverted and save to flash memory
    REVLibError resultFront = m_rightFrontMotor.burnFlash();
    REVLibError resultRear = m_rightRearMotor.burnFlash();
    if((resultFront != REVLibError.kOk) || (resultRear != REVLibError.kOk)){
      // Uh oh! Saving the inverted status to flash memory failed!
      // Action TBD
    }
    */
    
    m_leftFrontMotor = new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless);
    m_leftRearMotor = new CANSparkMax(5, CANSparkMaxLowLevel.MotorType.kBrushless);
     
    m_leftMotors = new MotorControllerGroup(m_leftFrontMotor, m_leftRearMotor);
    m_rightMotors = new MotorControllerGroup(m_rightFrontMotor, m_rightRearMotor);
    m_robotDrive = new DifferentialDrive(m_leftMotors, m_rightMotors);

    m_intakeMotor = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless);

    m_stick = new XboxController(0);

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
    /* smartdashboard */
    m_autoSelected = m_chooser.getSelected();
    m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);

    /* Set up the Timer for measuring how long we drive in autonomous */
    m_timer.reset();
    m_timer.start();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    /* smartdashboard integration */
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        // Drive for 2 seconds
        if (m_timer.get() < 3.0) {
          /**
          * Operate the robotDrive subclass to drive the robot
          */
          m_robotDrive.arcadeDrive(0.5, 0.0); // drive forwards half speed

        } else {
          m_robotDrive.stopMotor(); // stop robot
        }
        break; // Exit the case statement
    }
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
    double y_val = m_stick.getRightY();
    double x_val = m_stick.getLeftY();

    // Keep the sign for later on (+ for forward, - for backward)
    double y_sign = 1.0;
    double x_sign = 1.0;
    if(y_val < 0){
      y_sign = -y_sign;
    }
    if(x_val < 0){
      x_sign = -x_sign;
    }
    // Square the inputs for exponential response, but keep the sign
    m_robotDrive.tankDrive((y_sign * (y_val * y_val)),
                           (x_sign * (x_val * x_val)));
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
