/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera2;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/*
 * This is an advanced sample showcasing detecting and determining the orientation
 * of multiple stones, switching the viewport output, and communicating the results
 * of the vision processing to usercode.
 */
@TeleOp
public class openCV extends LinearOpMode
{
    OpenCvCamera webcam;
    openCV.BallCubeOpenCV pipeline;

    public static Rect[] bboundRect;
    public static Rect[] cboundRect;

    @Override
    public void runOpMode()
    {
        /**
         * NOTE: Many comments have been omitted from this sample for the
         * sake of conciseness. If you're just starting out with EasyOpenCv,
         * you should take a look at {@link InternalCamera2Example} or its
         * webcam counterpart, {@link WebcamExample} first.
         */

        // Create camera instance
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        // With live preview
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // Open async and start streaming inside opened callback
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

                pipeline = new openCV.BallCubeOpenCV(telemetry);
                webcam.setPipeline(pipeline);
            }
        });

        // Tell telemetry to update faster than the default 250ms period :)
        telemetry.setMsTransmissionInterval(250);

        waitForStart();

        while (opModeIsActive())
        {
            if(bboundRect != null) {
                telemetry.addData("Balls ", bboundRect.length);
            }
            else{
                telemetry.addData("Balls", " null");
            }
            if(cboundRect != null) {
                telemetry.addData("Cubes ", cboundRect.length);
            }
            else{
                telemetry.addData("Cubes", " null");
            }
            telemetry.update();
        }
    }


    public static class BallCubeOpenCV extends OpenCvPipeline {

        /*
         * These are our variables that will be
         * modifiable from the variable tuner.
         *
         * Scalars in OpenCV are generally used to
         * represent color. So our values in the
         * lower and upper Scalars here represent
         * the Y, Cr and Cb values respectively.
         *
         * YCbCr, like most color spaces, range
         * from 0-255, so we default to those
         * min and max values here for now, meaning
         * that all pixels will be shown.
         */
        public Scalar lower = new Scalar(0, 0, 0);
        public Scalar upper = new Scalar(255, 255, 255);



        /*
         * A good practice when typing EOCV pipelines is
         * declaring the Mats you will use here at the top
         * of your pipeline, to reuse the same buffers every
         * time. This removes the need to call mat.release()
         * with every Mat you create on the processFrame method,
         * and therefore, reducing the possibility of getting a
         * memory leak and causing the app to crash due to an
         * "Out of Memory" error.
         */
        private Mat ycrcbMat = new Mat();
        private Mat binaryMat = new Mat();
        private Mat maskedInputMat = new Mat();

        /*
         * Our working image buffers
         */
        Mat cbMat = new Mat();
        Mat yMat = new Mat();
        Mat cbthresholdMat = new Mat();
        Mat cbthresholdMatInv = new Mat();
        Mat ythresholdMat = new Mat();
        Mat morphedThreshold = new Mat();
        Mat contoursOnPlainImageMat = new Mat();

        /*
         * Threshold values
         */
        static final int CB_CHAN_MASK_THRESHOLD = 110;
        static final int Y_CHAN_MASK_THRESHOLD = 170;

        Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
        Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(6, 6));

        static final int CONTOUR_LINE_THICKNESS = 2;
        static final int Y_CHAN_IDX = 0;
        static final int CB_CHAN_IDX = 2;

        private Telemetry telemetry = null;



        public BallCubeOpenCV(Telemetry telemetry) {
            this.telemetry = telemetry;
        }

        @Override
        public Mat processFrame(Mat input) {
// A list we'll be using to store the contours we find

            // Convert the input image to YCrCb color space, then extract the Cb channel
            Imgproc.cvtColor(input, cbMat, Imgproc.COLOR_RGB2YCrCb);
            Imgproc.cvtColor(input, yMat, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(cbMat, cbMat, CB_CHAN_IDX);
            Core.extractChannel(yMat, yMat, Y_CHAN_IDX);

            // Threshold the Cb channel to form a mask, then run some noise reduction
            Imgproc.threshold(cbMat, cbthresholdMat, CB_CHAN_MASK_THRESHOLD, 255, Imgproc.THRESH_BINARY_INV);
            Imgproc.threshold(yMat, ythresholdMat, Y_CHAN_MASK_THRESHOLD, 255, Imgproc.THRESH_BINARY);

            Imgproc.erode(cbthresholdMat, cbthresholdMat, erodeElement);
            Imgproc.erode(cbthresholdMat, cbthresholdMat, erodeElement);

            Imgproc.dilate(cbthresholdMat, cbthresholdMat, dilateElement);
            Imgproc.dilate(cbthresholdMat, cbthresholdMat, dilateElement);

            Core.bitwise_not(cbthresholdMat, cbthresholdMatInv);
            Core.bitwise_and(ythresholdMat, cbthresholdMatInv, ythresholdMat);

            Imgproc.erode(ythresholdMat, ythresholdMat, erodeElement);
            Imgproc.erode(ythresholdMat, ythresholdMat, erodeElement);

            Imgproc.dilate(ythresholdMat, ythresholdMat, dilateElement);

            Imgproc.dilate(ythresholdMat, ythresholdMat, dilateElement);


            // A list we'll be using to store the contours we find
            ArrayList<MatOfPoint> contoursListballs = new ArrayList<>();
            ArrayList<MatOfPoint> contoursListcubes = new ArrayList<>();

            // Ok, now actually look for the contours! We only look for external contours.
            Imgproc.findContours(ythresholdMat, contoursListballs, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);

            Imgproc.findContours(cbthresholdMat, contoursListcubes, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);



            MatOfPoint2f[] bcontoursPoly  = new MatOfPoint2f[contoursListballs.size()];
            Rect[] bboundRect = new Rect[contoursListballs.size()];
            for (int i = 0; i < contoursListballs.size(); i++) {
                bcontoursPoly[i] = new MatOfPoint2f();
                Imgproc.approxPolyDP(new MatOfPoint2f(contoursListballs.get(i).toArray()), bcontoursPoly[i], 3, true);
                bboundRect[i] = Imgproc.boundingRect(new MatOfPoint(bcontoursPoly[i].toArray()));
                if(bboundRect[i].height > 100 || bboundRect[i].width > 100 ){
                    bboundRect[i] = null;
                }
            }


            MatOfPoint2f[] ccontoursPoly  = new MatOfPoint2f[contoursListcubes.size()];
            Rect[] cboundRect = new Rect[contoursListcubes.size()];
            for (int i = 0; i < contoursListcubes.size(); i++) {
                ccontoursPoly[i] = new MatOfPoint2f();
                Imgproc.approxPolyDP(new MatOfPoint2f(contoursListcubes.get(i).toArray()), ccontoursPoly[i], 3, true);
                cboundRect[i] = Imgproc.boundingRect(new MatOfPoint(ccontoursPoly[i].toArray()));
                if(cboundRect[i].height > 100 || cboundRect[i].width > 100 ){
                    cboundRect[i] = null;
                }
            }


            maskedInputMat.release();

            // We do draw the contours we find, but not to the main input buffer.
            input.copyTo(maskedInputMat);
            Imgproc.drawContours(maskedInputMat, contoursListballs, -1, new Scalar(255, 255, 255), CONTOUR_LINE_THICKNESS, 8);
            Imgproc.drawContours(maskedInputMat, contoursListcubes, -1, new Scalar(255, 255, 0), CONTOUR_LINE_THICKNESS, 8);


            for (int i = 0; i < contoursListballs.size(); i++) {
                Scalar color = new Scalar(255, 255, 255);
                if (bboundRect[i] != null) {
                    Imgproc.rectangle(maskedInputMat, bboundRect[i].tl(), bboundRect[i].br(), new Scalar(255, 255, 255), 2);
                }
            }

            for (int i = 0; i < contoursListcubes.size(); i++) {
                Scalar color = new Scalar(255, 255, 0);
                if (cboundRect[i] != null) {
                    Imgproc.rectangle(maskedInputMat, cboundRect[i].tl(), cboundRect[i].br(), new Scalar(255, 255, 0), 2);
                }
            }

            /*
             * Add some nice and informative telemetry messages
             */

            //telemetry.addData("Number of Balls", contoursListballs.size());
            //telemetry.addData("Number of Cubes", contoursListcubes.size());
            //telemetry.update();

            /*
             * The Mat returned from this method is the
             * one displayed on the viewport.
             *
             * To visualize our threshold, we'll return
             * the "masked input mat" which shows the
             * pixel from the input Mat that were inside
             * the threshold range.
             */
            return maskedInputMat;
        }
    }
}