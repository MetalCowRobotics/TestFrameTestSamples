package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  private final Joystick controller = new Joystick(0);
  private DriveTrain driveTrain = new DriveTrain();

  Servo arm = new Servo(1);

  // Motor values
  private double direction = 0, speed = 0, leftSpeed = 0, rightSpeed = 0;
  // Sensor values
  private double input1 = 0, input2 = 0, timeValue = 0;
  // Set this value what the value should be when the sensor goes off target.
  // Value shoud be between 0 and 4096, with 0 at 0V and 4096 at 5V.
  private double targetValue = 2000;
  // Amount which the system corrects for being off target, change as needed.
  private double correctionMult = .01;
  // testing
  boolean debugEnable = false;
  boolean simulateSensors = true;
  int debug = 0;
  // Line following
  double kP = 0;
  double kI = 0;
  double kD = 0;
  double proportional, derivative, lastDerivative, integral = 0;
  double onLine = 2500;
  double offLine = 1200;
  double lineTarget = (onLine + offLine) / 2;
  // PIDController rightEdge = new PIDController(kP, kI, kD, analogInput1,
  // driveTrain);
  LineSensor linePID = new LineSensor();

  @Override
  public void robotInit() {
    SmartDashboard.putNumber("OnLine", onLine);
    SmartDashboard.putNumber("OffLine", offLine);
    SmartDashboard.putNumber("P", kP);
    SmartDashboard.putNumber("I", kI);
    SmartDashboard.putNumber("D", kD);
    SmartDashboard.putNumber("speed", -.3);
    // rightEdge.setSetpoint(lineTarget);
  }

  @Override
  public void autonomousInit() {
    linePID.resetPID();
  }

  @Override
  public void autonomousPeriodic() {
    double speed = SmartDashboard.getNumber("speed", -.3);
    double correction = linePID.getCorrection();
    driveTrain.drive(speed, correction);
  }

  @Override
  public void teleopInit() {
    linePID.resetPID();
  }

  @Override
  public void teleopPeriodic() {
    // pushSensor();
    direction = controller.getX();
    speed = controller.getY();
    // if (linePID.onLine()) {
      if (controller.getRawButton(1)) {
      controller.setRumble(RumbleType.kRightRumble, 1);
      driveTrain.drive(SmartDashboard.getNumber("speed", -.3), linePID.getCorrection());
      // driveTrain.drive(SmartDashboard.getNumber("speed", -.3),
      // rightEdge.getError());
    } else {
      controller.setRumble(RumbleType.kRightRumble, 0);
      linePID.resetPID();
      driveTrain.drive(speed, direction);
      // arm.setBounds(max, deadbandMax, center, deadbandMin, min);
      // arm.set(controller.getX());
    }

    // lineFollowing();

    SmartDashboard.updateValues();
  }

  // Processes input for line following
  public void lineFollowing() {
    // The sensor numbers are arbitrary, but right now assume that the left is
    // sensor 1 and right is sensor 2.
    // To fix this just switch the input channels above or which motor they each
    // effect below.
    // input1 = analogInput1.getValue();
    // input2 = analogInput2.getValue();

    SmartDashboard.putNumber("Analog input 1", input1);
    SmartDashboard.putNumber("Analog input 2", input2);

    // TODO: Account for inital crossing of line, currrently assumes it started with
    // the sensors stradling the line.

    // Sensors should be slightly wider than line by default, so sensors should be
    // off target when following line.
    // Might need to reverse signs, I dont know if it goes left or right if it is
    // off target
    if (input1 > targetValue) { // If left sensor is over the target then go right.
      timeValue++; // Strengthens the correction over time.
      direction += direction * correctionMult;
      // rightSpeed -= timeValue * correctionMult;
    } else if (input2 > targetValue) { // If right sensor is over the target then go left.
      timeValue++; // Strengthens the correction over time.
      direction -= direction * correctionMult;
      // rightSpeed += timeValue * correctionMult;
    }
  }

  @Override
  public void testPeriodic() {
  }
}
