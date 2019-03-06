package frc.robot; 

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class DriveTrain implements PIDOutput {

    private MCR_SRX leftDrive = new MCR_SRX(1);
    private MCR_SRX rightDrive = new MCR_SRX(10);
    private DifferentialDrive driveTrain = new DifferentialDrive(leftDrive, rightDrive);
    private double savedSpeed = -.5;

    public void drive(double speed, double direction) {
        driveTrain.arcadeDrive(speed, direction);
    }

	@Override
	public void pidWrite(double output) {
		drive(savedSpeed, output);
	}

}