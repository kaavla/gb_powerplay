package org.firstinspires.ftc.teamcode.whiteout.RobotDrivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;


public class RobotDrivetrainHW {

    public DcMotor leftMotor      = null;
    public DcMotor rightMotor     = null;
    public DcMotor backrightMotor = null;
    public DcMotor backleftMotor  = null;

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        leftMotor      = ahwMap.get(DcMotor.class, "M1");
        rightMotor     = ahwMap.get(DcMotor.class, "M2");
        backleftMotor  = ahwMap.get(DcMotor.class, "M3");
        backrightMotor = ahwMap.get(DcMotor.class, "M4");

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backleftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setPowerAll(double fl_power, double fr_power, double bl_power, double br_power){
        leftMotor.setPower(fl_power);
        rightMotor.setPower(fr_power);
        backleftMotor.setPower(bl_power);
        backrightMotor.setPower(br_power);
    }

    public void stopAllMotors() {
        setPowerAll(0.0, 0.0, 0.0, 0.0);
    }

    public void moveHolonomic(double x, double y , double z)
    {
        double max_power = 0.6;
        double min_power = -1*max_power;

        double fl_power = Range.clip(y + x - z, min_power, max_power);
        double fr_power = Range.clip(y - x + z, min_power, max_power);
        double br_power = Range.clip(y + x + z, min_power, max_power);
        double bl_power = Range.clip(y - x - z, min_power, max_power);

        setPowerAll(fl_power, fr_power, bl_power, br_power);
    }

    public void forwardSlow() {
        double power = Range.clip(leftMotor.getPower() + 0.01, 0.3, 1.0);
        setPowerAll(power, power, power, power);
    }

    public void backwardSlow() {
        double power = Range.clip(leftMotor.getPower() - 0.01, -0.3, -1.0);
        setPowerAll(power, power, power, power);
    }
    public int getEncoderPos(int wheel){
        int pos = 0;
        if (wheel == 1) {
            pos  = leftMotor.getCurrentPosition();
        }else if (wheel == 2) {
            pos  = rightMotor.getCurrentPosition();
        }else if (wheel == 3) {
            pos  = backleftMotor.getCurrentPosition();
        }else{
            pos = backrightMotor.getTargetPosition();
        }
        return  pos;
    }

}

