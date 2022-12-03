package org.firstinspires.ftc.teamcode.whiteout.OpenCVDetector;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class IntakeCapstoneDetectorOnlyBlue extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();
    Mat white_mat = new Mat();

    public enum IntakeStatus {
        EMPTY,
        FULL
    }
    private IntakeStatus intake_status = IntakeStatus.EMPTY;

    static final Rect CAP_ROI = new Rect(
            new Point(120, 140),
            new Point(420, 440));
    static double PERCENT_COLOR_THRESHOLD = 0.2;
    private boolean debug_mode = true;
    public IntakeCapstoneDetectorOnlyBlue(Telemetry t) {
        telemetry = t;
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        //Blue
        //Scalar lowHSV = new Scalar(120, 50, 50);
        //Scalar highHSV = new Scalar(140, 255, 255);

        Scalar lowHSV = new Scalar(100, 20, 30);
        Scalar highHSV = new Scalar(130, 255, 255);

        Core.inRange(mat, lowHSV, highHSV, mat);

        Mat left = mat.submat(CAP_ROI);

        double leftValue = Core.sumElems(left).val[0] / CAP_ROI.area() / 255;

        left.release();

        if (debug_mode) {
            telemetry.addData("Left raw value", (int) Core.sumElems(left).val[0]);
            telemetry.addData("Left percentage", Math.round(leftValue * 100) + "%");
        }
        boolean capLeft = leftValue < PERCENT_COLOR_THRESHOLD;

        Scalar colorNotCap = new Scalar(255, 0, 0);
        Scalar colorCap = new Scalar(0, 255, 0);

        if (capLeft ) {
            intake_status = IntakeStatus.FULL;
            if (debug_mode) telemetry.addData("Capstone Location", "FULL");
            Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);
            Imgproc.rectangle(mat, CAP_ROI, intake_status == IntakeStatus.FULL? colorCap:colorNotCap);
        } else {
            intake_status = IntakeStatus.EMPTY;
            if (debug_mode) telemetry.addData("Capstone Location", "EMPTY");
            Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);
            Imgproc.rectangle(mat, CAP_ROI, intake_status == IntakeStatus.FULL? colorCap:colorNotCap);
        }
        if (debug_mode) telemetry.update();

        return mat;
    }

    public IntakeStatus getIntakeStatus() {
        return intake_status;
    }
}