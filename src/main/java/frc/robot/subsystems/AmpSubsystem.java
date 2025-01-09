package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.States.*;
import frc.robot.Constants.*;
import frc.robot.Constants.FullAmpSubsystem.InfoAmpSubsystem; 
import frc.robot.Constants.FullAmpSubsystem.ArmPIDValues; 

public class AmpSubsystem {
    CANSparkMax LeftMotor; 
    CANSparkMax RightMotor; 
    CANSparkMax TrapMotor;
    
    private PIDController arm_pid; 
    private double armPosition;
    private DutyCycleEncoder armEncoder;
    private double armspeed; 
    private double targetPos; 

    public AmpSubsystem(){
        LeftMotor = new CANSparkMax(InfoAmpSubsystem.ArmleftMotorID, MotorType.kBrushless); 
        RightMotor = new CANSparkMax(InfoAmpSubsystem.ArmRightMotorID, MotorType.kBrushless); 
        TrapMotor = new CANSparkMax(InfoAmpSubsystem.TrapID, MotorType.kBrushless); 

        arm_pid = new PIDController(ArmPIDValues.ArmPID_P, ArmPIDValues.ArmPID_I, ArmPIDValues.ArmPID_D); 
        armPosition = armEncoder.getAbsolutePosition() * 360;
    }

    public void setTrapMotorStates (TrapMotorState TrapState) {
        switch (TrapState) {
            case IN: 
                setAmpSpeed(0, 0, 0.75);
                break; 
            case OUT: 
                setAmpSpeed(0, 0, -0.75);
                break; 
            case STOP: 
                setAmpSpeed(0, 0, 0);
                break; 
            default: 
                break; 
        }
    }

    public void setArmStates (ArmMotorState ArmState) {
        switch (ArmState) {
            case ANGLE1: 
                armspeed = setPIDTargetedPos(0.5);
                SetSmartDashboard();
                setAmpSpeed(armspeed, -armspeed, 0);
                break; 
            case STOP: 
                setAmpSpeed(0, 0, 0);
                break; 
            default: 
                break; 
        }
    }

    public void setAmpSpeed(double leftMotorSpeed, double RightMotorSpeed, double TrapMotorSpeed) {
        LeftMotor.set(leftMotorSpeed);
        RightMotor.set(RightMotorSpeed);
        TrapMotor.set(TrapMotorSpeed);
    }

    public double setPIDTargetedPos(double target){
        arm_pid.setSetpoint(target);
        double speed = arm_pid.calculate(armPosition, targetPos); 

        return speed; 
    }

    public void SetSmartDashboard(){
        SmartDashboard.putNumber("Amp Arm Postion", armPosition);
        SmartDashboard.putNumber("Amp Arm Speed/Output", armspeed); 
    }
}
