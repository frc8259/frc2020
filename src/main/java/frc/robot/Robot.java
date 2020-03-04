package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Robot extends TimedRobot {
  public final PWMVictorSPX leftMotor = new PWMVictorSPX(0);
  public final PWMVictorSPX rightMotor = new PWMVictorSPX(1);

  public final DifferentialDrive drive = new DifferentialDrive(leftMotor, rightMotor);

  public final PWMVictorSPX intakeMotor = new PWMVictorSPX(2);
  public final PWMVictorSPX beltMotor = new PWMVictorSPX(3);

  public final PWMVictorSPX leftLift = new PWMVictorSPX(4);
  public final PWMVictorSPX rightLift = new PWMVictorSPX(5);

  public final SpeedControllerGroup liftMotor = new SpeedControllerGroup(leftLift, rightLift);

  public final DigitalInput topLeftLift = new DigitalInput(1);
  public final DigitalInput bottomLeftLift = new DigitalInput(3);

  public final DigitalInput topRightLift = new DigitalInput(2);
  public final DigitalInput bottomRightLift = new DigitalInput(4);

  public final Joystick driver = new Joystick(0);
  public final Joystick operator = new Joystick(1);

  public final Timer timer = new Timer();
  public final AHRS gyro = new AHRS(SPI.Port.kMXP);

  double p = 0.01;
  double i = 0.01;
  double d = 0.01;

  @Override
  public void robotInit() {
    CameraServer.getInstance().startAutomaticCapture().setResolution(320, 240);

    gyro.calibrate();
    gyro.reset();
  }

  @Override
  public void robotPeriodic() {
    System.out.print("Gyro: ");
    System.out.println(Math.round(gyro.getAngle()));
    System.out.print("Accel: ");
    System.out.println(Math.round(gyro.getVelocityY()));
  }

  @Override
  public void autonomousInit() {
    timer.reset();
    timer.start();
  }

  @Override
  public void autonomousPeriodic() {
    if (timer.get() < 1) {
      double error = -gyro.getRate();
      drive.tankDrive(-0.5 + p * error, -0.5 - p * error);
    } else if (timer.get() < 1 + 2) {
      beltMotor.set(0.2);
    } else if (timer.get() < 1 + 2 + 1) {
      double error = -gyro.getRate();
      drive.tankDrive(0.5 + p * error, 0.5 - p * error);
    } else if (timer.get() < 1 + 2 + 1 + 0.5) {
      double heading = -90 - gyro.getAngle();
      drive.tankDrive(p * heading, p * heading);
    } else if (timer.get() < 1 + 2 + 1 + 0.5 + 1) {
      double error = -gyro.getRate();
      drive.tankDrive(0.5 + p * error, 0.5 - p * error);
    } else if (timer.get() < 1 + 2 + 1 + 0.5 + 1 + 0.5) {
      double heading = 0 - gyro.getAngle();
      drive.tankDrive(p * heading, p * heading);
    } else {
      drive.stopMotor();
      beltMotor.stopMotor();
    }
  }

  @Override
  public void teleopInit() {
    timer.reset();
    timer.start();
  }

  @Override
  public void teleopPeriodic() {
    drive.arcadeDrive(driver.getY(), driver.getZ() * 0.5);

    if (operator.getRawButton(1)) {
      intakeMotor.set(1);
    } else if (operator.getRawButton(2)) {
      intakeMotor.set(-1);
    } else {
      intakeMotor.stopMotor();
    }

    if (operator.getPOV() == 180) {
      beltMotor.set(1);
    } else if (operator.getPOV() == 0) {
      beltMotor.set(-1);
    } else {
      beltMotor.stopMotor();
    }

    if (operator.getRawAxis(1) < 0 & topLeftLift.get()) {
      leftLift.set(operator.getRawAxis(1));
    } else if (operator.getRawAxis(1) > 0 & bottomLeftLift.get()) {
      leftLift.set(operator.getRawAxis(1));
    } else {
      leftLift.stopMotor();
    }

    if (operator.getRawAxis(5) < 0 & topRightLift.get()) {
      rightLift.set(operator.getRawAxis(5));
    } else if (operator.getRawAxis(5) > 0 & bottomRightLift.get()) {
      rightLift.set(operator.getRawAxis(5));
    } else {
      rightLift.stopMotor();
    }
  }

  @Override
  public void testInit() {
    timer.reset();
    timer.start();
  }

  @Override
  public void testPeriodic() {
    double error = -gyro.getRate();
    drive.tankDrive(0.5 + p * error, 0.5 - p * error);

    double heading = 90 - gyro.getAngle();
    drive.tankDrive(p * heading, p * heading);
  }
}