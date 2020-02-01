package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Arm implements Subsystem {

    private static TalonSRX motor = new TalonSRX(8);
    
    private static ArmFeedforward feedforward = new ArmFeedforward(0.449, 0.00354, 0.00105);
    private static PIDController pidcontroller = new PIDController(0.00521, 0, 0.00166);

    public Arm () {
        motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);
    }

    public void resetEncoder() {
        motor.setSelectedSensorPosition(0);
    }

    public int getEncoderPosition() {
        return motor.getSelectedSensorPosition();
    }

    public void setPosition(double position) {
        double output = 0;

        output += feedforward.calculate(position, 0);
        double error = position - getEncoderPosition()/4096*(2*Math.PI);
        output += pidcontroller.calculate(error);

        SmartDashboard.putNumber("error", error);
        SmartDashboard.putNumber("encoder", getEncoderPosition());
        
        motor.set(ControlMode.PercentOutput, output);
    }
}