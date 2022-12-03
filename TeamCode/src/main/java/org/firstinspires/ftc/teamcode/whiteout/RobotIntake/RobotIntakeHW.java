package org.firstinspires.ftc.teamcode.whiteout.RobotIntake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class RobotIntakeHW {

    public DcMotor I0      = null;


    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        I0      = ahwMap.get(DcMotor.class, "I0");

    }

    public void setPower(double power){
        I0.setPower(power);
    }

    public double getPower(){
        return I0.getPower();
    }

    public void stopIntake() {
        setPower(0.0);
    }
    public void startIntake(double power) {
        setPower(power);
    }


}

