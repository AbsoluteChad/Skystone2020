package org.firstinspires.ftc.teamcode.lib.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class SkystoneDetector {

    //Values of the 3 blocks (skystone or not)
    private static int valLeft = -1;
    private static int valMiddle = -1;
    private static int valRight = -1;

    //Dimensions of rectangles on screen
    private static float rectHeight = .6f/8f;
    private static float rectWidth = 1.5f/8f;

    private static float offsetX = 0f/8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static float offsetY = 0f/8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

    //Position of rectangles on screen
    private static float[] leftPos = {2f/8f + offsetX, 4f/8f + offsetY}; //0 = col, 1 = row
    private static float[] middlePos = {4f/8f + offsetX, 4f/8f + offsetY};
    private static float[] rightPos = {6f/8f + offsetX, 4f/8f + offsetY};

    //Screen dimensions
    private final int rows = 640;
    private final int cols = 480;

    //Declare camera
    private OpenCvCamera phoneCamera;

    public SkystoneDetector(HardwareMap hardwareMap) {
        //Init phone camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCamera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        //Start pipeline
        openCamera();
        setPipeline(new SkystonePipeline());
        startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);
    }

    /**
     * Gets the position of the skystone (assuming phone camera sees 3 blocks in which one is a skystone)
     * @return the skystone position as a char 'C', 'M', or 'F' for "Close" "Middle" and "Far" (to the foundation). Will return
     * 'N' if the skystone cannot properly be detected. Keep in mind that C, M, and F are in different directions depending on the
     *  alliance color
     */
    public char getSkystonePosition() {
        if (valLeft == 0) {
            return 'C';
        } else if (valMiddle == 0) {
            return 'M';
        } else if (valRight == 0) {
            return 'F';
        }
        return 'N';
    }

    /**
     * Opens camera device
     */
    public void openCamera() {
        phoneCamera.openCameraDevice();
    }

    /**
     * Closes camera device
     */
    public void closeCamera() {
        phoneCamera.closeCameraDevice();
    }

    /**
     * Sets pipeline for vision
     * @param pipeline desired vision pipeline
     */
    public void setPipeline(OpenCvPipeline pipeline) {
        phoneCamera.setPipeline(pipeline);
    }

    /**
     * Starts viewport to Driver Station phone
     * @param camRotation rotation of camera mounted on robot
     */
    public void startStreaming(int rows, int cols, OpenCvCameraRotation camRotation) {
        phoneCamera.startStreaming(rows, cols, camRotation);
    }

    /**
     * Pauses viewport to driver station phone
     * NOTE: pipeline does not stop
     */
    public void pauseViewport() {
        phoneCamera.pauseViewport();
    }

    /**
     * Resumes viewport to driver station phone (if paused)
     */
    public void resumeViewport() {
        phoneCamera.resumeViewport();
    }

    /**
     * @return the camera FPS
     */
    public double getFPS() {
        return phoneCamera.getFps();
    }

    /**
     * Pipeline class used for detecting skystone
     */
    private static class SkystonePipeline extends OpenCvPipeline {
        Mat yCbCrChan2Mat = new Mat();
        Mat thresholdMat = new Mat();
        Mat all = new Mat();
        List<MatOfPoint> contoursList = new ArrayList<>();

        enum Stage {
            DETECTION,//includes outlines
            THRESHOLD,//b&w
            RAW_IMAGE,//displays raw view
        }

        private SkystoneDetectorOpMode.StageSwitchingPipeline.Stage stageToRenderToViewport = SkystoneDetectorOpMode.StageSwitchingPipeline.Stage.DETECTION;
        private SkystoneDetectorOpMode.StageSwitchingPipeline.Stage[] stages = SkystoneDetectorOpMode.StageSwitchingPipeline.Stage.values();

        @Override
        public void onViewportTapped() {
            /*
             * Note that this method is invoked from the UI thread
             * so whatever we do here, we must do quickly.
             */

            int currentStageNum = stageToRenderToViewport.ordinal();
            int nextStageNum = currentStageNum + 1;
            if(nextStageNum >= stages.length) {
                nextStageNum = 0;
            }
            stageToRenderToViewport = stages[nextStageNum];
        }

        @Override
        public Mat processFrame(Mat input) {
            contoursList.clear();
            /*
             * This pipeline finds the contours of yellow blobs such as the Gold Mineral
             * from the Rover Ruckus game.
             */

            //color diff cb.
            //lower cb = more blue = skystone = white
            //higher cb = less blue = yellow stone = grey
            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);//takes cb difference and stores

            //b&w
            Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);

            //outline/contour
            Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            yCbCrChan2Mat.copyTo(all);//copies mat object
            //Imgproc.drawContours(all, contoursList, -1, new Scalar(255, 0, 0), 3, 8);//draws blue contours


            //get values from frame
            double[] pixMid = thresholdMat.get((int)(input.rows()* middlePos[1]), (int)(input.cols()* middlePos[0]));//gets value at circle
            valMiddle = (int)pixMid[0];

            double[] pixLeft = thresholdMat.get((int)(input.rows()* leftPos[1]), (int)(input.cols()* leftPos[0]));//gets value at circle
            valLeft = (int)pixLeft[0];

            double[] pixRight = thresholdMat.get((int)(input.rows()* rightPos[1]), (int)(input.cols()* rightPos[0]));//gets value at circle
            valRight = (int)pixRight[0];

            //create three points
            Point pointMid = new Point((int)(input.cols()* middlePos[0]), (int)(input.rows()* middlePos[1]));
            Point pointLeft = new Point((int)(input.cols()* leftPos[0]), (int)(input.rows()* leftPos[1]));
            Point pointRight = new Point((int)(input.cols()* rightPos[0]), (int)(input.rows()* rightPos[1]));

            //draw circles on those points
            Imgproc.circle(all, pointMid,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointLeft,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointRight,5, new Scalar( 255, 0, 0 ),1 );//draws circle

            //draw 3 rectangles
            Imgproc.rectangle(//1-3
                    all,
                    new Point(
                            input.cols()*(leftPos[0]-rectWidth/2),
                            input.rows()*(leftPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(leftPos[0]+rectWidth/2),
                            input.rows()*(leftPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//3-5
                    all,
                    new Point(
                            input.cols()*(middlePos[0]-rectWidth/2),
                            input.rows()*(middlePos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(middlePos[0]+rectWidth/2),
                            input.rows()*(middlePos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//5-7
                    all,
                    new Point(
                            input.cols()*(rightPos[0]-rectWidth/2),
                            input.rows()*(rightPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(rightPos[0]+rectWidth/2),
                            input.rows()*(rightPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);

            switch (stageToRenderToViewport) {
                case THRESHOLD: return thresholdMat;
                case DETECTION: return all;
                case RAW_IMAGE: return input;
                default: return input;
            }
        }
    }
}
