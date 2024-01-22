package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.utils.MotorUtil;

public class RealArm implements ArmIO {
    private static CANSparkMax armMotorControllerLeft;
    private static CANSparkMax armMotorControllerRight;
    public static RelativeEncoder armEncoder;
    public static DutyCycleEncoder armAbsoluteEncoder;

    public RealArm() {
        // armAbsoluteEncoder = new DutyCycleEncoder(0);
        //Higher slew rate of .75 seconds from 0 to 100% (sparkmax thinks we use this) translates to .2 seconds from 0 to 20% (what we actually use)
        armMotorControllerLeft = MotorUtil.createSparkMAX(7, MotorType.kBrushless, Constants.NEO_CURRENT_LIMIT, 
            true, true, 0.75);
         armMotorControllerRight = MotorUtil.createSparkMAX(6, MotorType.kBrushless, Constants.NEO_CURRENT_LIMIT, 
            true, true, 0.75);
        armEncoder = armMotorControllerLeft.getEncoder();
        
        armEncoder.setPositionConversionFactor(ArmConstants.RADIANS_PER_REVOLUTION);
        armEncoder.setVelocityConversionFactor(ArmConstants.RADIANS_PER_REVOLUTION / 60);

        armEncoder.setPosition(ArmConstants.INITIAL_OFFSET);
    }

    public double getAbsoluteEncoderPosition() {
        return -(armAbsoluteEncoder.getAbsolutePosition() - 0.88) * 2 * Math.PI / 3;
    }

    @Override
    public void periodicUpdate() {
        SmartDashboard.putNumber("arm/temp (C)", armMotorControllerLeft.getMotorTemperature());
    }

    @Override
    public double getEncoderPosition() {
       //return getAbsoluteEncoderPosition();
        return armEncoder.getPosition();
    }

    @Override
    public double getEncoderSpeed() {
        return armEncoder.getVelocity();
    }

    @Override
    public void setSpeed(double speed) {
        armMotorControllerLeft.set(speed);
    }

    @Override
    public void setPosition(double position) {
        armEncoder.setPosition(position);
    }

    @Override
    public double getArmCurrent() {
        return armMotorControllerLeft.getOutputCurrent();
    }
    
}
