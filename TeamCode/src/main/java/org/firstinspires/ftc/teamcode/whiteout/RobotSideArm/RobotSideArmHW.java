package org.firstinspires.ftc.teamcode.whiteout.RobotSideArm;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

public class RobotSideArmHW {
    public Servo SL; //base servo 1
    public Servo SR; //base servo 2

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        RobotLog.ii("L124", "Enter - init");

        SL  = ahwMap.get(Servo.class, "S3" );
        SR  = ahwMap.get(Servo.class, "CRS1" );

        initRobotArm();
        RobotLog.ii("L124", "Exit - init");
    }

    //Initializes the Robot Arm to given position
    public void initRobotArm() {
        SL.setDirection(Servo.Direction.REVERSE);
        //SR.setDirection(Servo.Direction.REVERSE);
        SL.setPosition(0.0);
        /*
        try {
            Thread.sleep(400);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }*/

        SR.setPosition(0.0);

    }

    public void servoSetPosRaw(int servonum, double newPos) {
        if (servonum == 0) {
            SL.setPosition(newPos);
        } else {
            SR.setPosition(newPos);
        }
    }

}

