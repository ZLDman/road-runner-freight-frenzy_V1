/*
 * Copyright (c) 2021 Sebastian Erives
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

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;


public class BallCubeOpenCV extends OpenCvPipeline {

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
     * This will allow us to choose the color
     * space we want to use on the live field
     * tuner instead of hardcoding it
     */
    public ColorSpace colorSpace = ColorSpace.YCrCb;

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
    private Mat ycrcbMat       = new Mat();
    private Mat binaryMat      = new Mat();
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

    /*
     * Enum to choose which color space to choose
     * with the live variable tuner isntead of
     * hardcoding it.
     */
    enum ColorSpace {
        /*
         * Define our "conversion codes" in the enum
         * so that we don't have to do a switch
         * statement in the processFrame method.
         */
        RGB(Imgproc.COLOR_RGBA2RGB),
        HSV(Imgproc.COLOR_RGB2HSV),
        YCrCb(Imgproc.COLOR_RGB2YCrCb),
        Lab(Imgproc.COLOR_RGB2Lab);

        //store cvtCode in a public var
        public int cvtCode = 0;

        //constructor to be used by enum declarations above
        ColorSpace(int cvtCode) {
            this.cvtCode = cvtCode;
        }
    }

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
        Core.bitwise_and(ythresholdMat,cbthresholdMatInv,ythresholdMat);

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
        }


        MatOfPoint2f[] ccontoursPoly  = new MatOfPoint2f[contoursListcubes.size()];
        Rect[] cboundRect = new Rect[contoursListcubes.size()];
        for (int i = 0; i < contoursListcubes.size(); i++) {
            ccontoursPoly[i] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contoursListcubes.get(i).toArray()), ccontoursPoly[i], 3, true);
            cboundRect[i] = Imgproc.boundingRect(new MatOfPoint(ccontoursPoly[i].toArray()));
        }


        maskedInputMat.release();

        // We do draw the contours we find, but not to the main input buffer.
        input.copyTo(maskedInputMat);
        Imgproc.drawContours(maskedInputMat, contoursListballs, -1, new Scalar(255, 255, 255), CONTOUR_LINE_THICKNESS, 8);
        Imgproc.drawContours(maskedInputMat, contoursListcubes, -1, new Scalar(255, 255, 0), CONTOUR_LINE_THICKNESS, 8);


        for (int i = 0; i < contoursListballs.size(); i++) {
            Scalar color = new Scalar(255, 255, 255);
            Imgproc.rectangle(maskedInputMat, bboundRect[i].tl(), bboundRect[i].br(), color, 2);
        }

        for (int i = 0; i < contoursListcubes.size(); i++) {
            Scalar color = new Scalar(255, 255, 0);
            Imgproc.rectangle(maskedInputMat, cboundRect[i].tl(), cboundRect[i].br(), color, 2);
        }

        /*
         * Add some nice and informative telemetry messages
         */

        telemetry.addData("Number of Balls", contoursListballs.size());
        telemetry.addData("Number of Cubes", contoursListcubes.size());
        telemetry.update();

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
