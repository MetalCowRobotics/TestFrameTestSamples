package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//testing the line follow

public class LineSensor {
    private AnalogInput leftSensor = new AnalogInput(0);
    // Line following
    private double kP = 0.0006;
    private double kI = 0;
    private double kD = 0.001;
    private double proportional, derivative, lastDerivative, integral = 0;
    private double onLine = 2500;
    private double offLine = 1200;
    private double lineTarget = (onLine + offLine) / 2;

    public LineSensor() {
        SmartDashboard.putNumber("OnLine", onLine);
        SmartDashboard.putNumber("OffLine", offLine);
        SmartDashboard.putNumber("P", kP);
        SmartDashboard.putNumber("I", kI);
        SmartDashboard.putNumber("D", kD);
        SmartDashboard.putNumber("speed", -.3);
        initializeLineFollower();
    }

    public double getCorrection() {
        pushSensor();
        proportional = lineTarget - leftSensor.getValue();
        integral = integral + proportional;
        derivative = proportional - lastDerivative;
        lastDerivative = proportional;
        kP = SmartDashboard.getNumber("P", kP);
        kI = SmartDashboard.getNumber("I", kI);
        kD = SmartDashboard.getNumber("D", kD);
        double correction = (proportional * kP) + (integral * kI) + (derivative * kD);
        pushPIDValues(correction);
        return correction;
    }

	private void pushPIDValues(double correction) {
		SmartDashboard.putNumber("proportinonal", proportional);
        SmartDashboard.putNumber("derative", derivative);
        SmartDashboard.putNumber("integral", integral);
        SmartDashboard.putNumber("correction", correction);
    }

    private void initializeLineFollower() {
        onLine = SmartDashboard.getNumber("OnLine", onLine);
        offLine = SmartDashboard.getNumber("OffLine", offLine);
        lineTarget = (onLine + offLine) / 2;
    }

    private void pushSensor() {
        SmartDashboard.putNumber("sensor", leftSensor.getValue());
    }

    public boolean onLine() {
        return leftSensor.getValue() > lineTarget;
      }

    public void resetPID() {
        proportional = 0;
        integral = 0;
        derivative = 0;
        lastDerivative = 0;
        initializeLineFollower();
      }

}