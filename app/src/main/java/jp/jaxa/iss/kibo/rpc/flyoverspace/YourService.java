package jp.jaxa.iss.kibo.rpc.flyoverspace;

import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.Kinematics;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.aruco.Aruco;
import org.opencv.aruco.Dictionary;
import org.opencv.aruco.DetectorParameters;
import java.util.*;
import java.util.concurrent.*;

/**
 * class meant to handle commands from the Ground Data System and execute them in Astrobee.
 */

public class YourService extends KiboRpcService implements ApiService {
    private static final double MAX_VELOCITY = Helper.MAX_VELOCITY;  // m/s
    private static final double MIN_DISTANCE = Helper.MIN_DISTANCE; // m
    private static final double MIN_ANGLE = Helper.MIN_ANGLE;     // degrees
    private static final double MAX_THRUST_X = Helper.MAX_THRUST_X;  // N
    private static final double ROBOT_MASS = Helper.ROBOT_MASS;   // kg

    private static final double[] KIZ1_BOUNDS = Helper.KIZ1_BOUNDS;
    private static final double[] KIZ2_BOUNDS = Helper.KIZ2_BOUNDS;
    private static final double[][] KOZ_BOUNDS = Helper.KOZ_BOUNDS;

    private final Point[] safeWaypoints = Helper.SAFE_WAYPOINTS;
    private final Point[] areaPoints = Helper.AREA_POINTS;
    
    private final Point startPoint = Helper.START_POINT;
    private final Point astronautPoint = Helper.ASTRONAUT_POINT;
    private final Quaternion startQuaternion = Helper.START_QUATERNION;
    private final Quaternion astronautQuaternion = Helper.ASTRONAUT_QUATERNION;
    
    private HashMap<Integer, String> foundItemsMap = new HashMap<>();
    private int targetAreaId = -1;

    private final Map<Integer, String> markerToItem = Helper.MARKER_TO_ITEM;

    private static final double[] NAV_CAM_OFFSET = Helper.NAV_CAM_OFFSET;
    private static final double[] HAZ_CAM_OFFSET = Helper.HAZ_CAM_OFFSET;
    private Mat cameraMatrix;
    private Mat distCoeffs;

    private final Map<String, Mat> imageCache = new ConcurrentHashMap<>();
    private final Map<Integer, Long> lastDetectionTime = new ConcurrentHashMap<>();
    private static final long DETECTION_COOLDOWN = Helper.DETECTION_COOLDOWN; // ms

    private final ExecutorService executor = Executors.newFixedThreadPool(2);
    private final CompletableFuture<Void> imageProcessingFuture = new CompletableFuture<>();

    private Mat grayMat;
    private Mat tempMat;
    private Dictionary dictionary;
    private final List<Mat> corners = new ArrayList<>();

    private static final int MAX_CACHE_SIZE = 5;
    private static final long MAX_CACHE_AGE = 5000; // 5 seconds

    private static class MatPool {
        private static final int POOL_SIZE = 10;
        private static final Queue<Mat> pool = new ConcurrentLinkedQueue<>();
        
        public static Mat acquire() {
            Mat mat = pool.poll();
            return mat != null ? mat : new Mat();
        }
        
        public static void release(Mat mat) {
            if (mat != null && !mat.empty()) {
                mat.release();
                if (pool.size() < POOL_SIZE) {
                    pool.offer(new Mat());
                }
            }
        }
        
        public static void clearPool() {
            while (!pool.isEmpty()) {
                Mat mat = pool.poll();
                if (mat != null) mat.release();
            }
        }
    }

    @Override
    protected void runPlan1() {
        try {
            initializeCameraParameters();
            executeMainMission();
        } catch (Exception e) {
            e.printStackTrace();
            handleMissionFailure();
        } finally {
            releaseMatObjects();
            cleanup();
        }
    }

    private void initializeCameraParameters() {
        double[][] camParams = api.getNavCamIntrinsics();
        cameraMatrix = new Mat(3, 3, CvType.CV_64F);
        distCoeffs = new Mat(1, 5, CvType.CV_64F);
        Helper.initializeCameraParameters(cameraMatrix, distCoeffs, camParams);
    }

    private void executeMainMission() {
        Helper.executeMainMission(this, startPoint, startQuaternion, areaPoints, executor);
    }

    private Point optimizeTargetPoint(Point originalTarget) {
        return Helper.optimizeTargetPoint(originalTarget, this);
    }

    public Quaternion createOptimizedScanningQuaternion(Point target) {
        return Helper.createOptimizedScanningQuaternion(target, this);
    }

    private boolean scanAreaOptimized(int areaIndex, DetectorParameters parameters) {
        return Helper.scanAreaOptimized(this, areaIndex, areaPoints, executor, this);
    }

    public List<Integer> detectArUcoMarkers(Mat image, String debugTag) {
        if (grayMat == null) grayMat = new Mat();
        if (dictionary == null) {
            dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
        }
        
        return Helper.detectArUcoMarkers(this, image, dictionary, grayMat, tempMat, corners);
    }

    private void handleMissionFailure() {
        Helper.handleMissionFailure(this);
    }

    private void cleanup() {
        Helper.cleanup(this, executor, imageCache, cameraMatrix, distCoeffs);
    }

    public boolean isValidMovement(Point point, Quaternion quat) {
        return Helper.isValidMovement(point, quat, this);
    }

    private Point findNearestSafePoint(Point currentPos) {
        Point nearest = safeWaypoints[0];
        double minDist = Double.MAX_VALUE;
        
        for (Point waypoint : safeWaypoints) {
            double dist = Helper.distance(waypoint, currentPos);
            if (dist < minDist && Helper.isPointSafe(waypoint)) {
                minDist = dist;
                nearest = waypoint;
            }
        }
        return nearest;
    }

    //helper mt to create scanning quaternion based on position
    public Quaternion createScanningQuaternion(Point target) {
        return Helper.createScanningQuaternion(target, startPoint);
    }

    //helper mt to handle movement with retry logic
    public void moveToWithRetry(Point targetPoint, Quaternion targetQuaternion) {
        Helper.moveToWithRetry(this, targetPoint, targetQuaternion);
    }

    @Override
    protected void runPlan2() {
        // Not used in preliminary round
    }

    @Override
    protected void runPlan3() {
        // Not used in preliminary round
    }

    private void saveDebugImage(Mat image, String tag, int attempt) {
        Helper.saveDebugImage(this, image, tag, attempt);
    }

    private void completeTargetOperationOptimized() {
        Helper.completeTargetOperationOptimized(
            this,
            astronautPoint,
            astronautQuaternion,
            targetAreaId,
            areaPoints,
            this
        );
    }

    private List<Point> findPathAStar(Point start, Point goal) {
        return Helper.findPathAStar(start, goal, this);
    }

    private void handleEmergencyStop() {
        Kinematics currentKinematics = api.getRobotKinematics();
        Helper.handleEmergencyStop(this, currentKinematics.getPosition(), 
                                 currentKinematics.getOrientation());
        cleanup();
    }

    private void releaseMatObjects() {
        Helper.releaseMatObjects(grayMat, tempMat, corners);
        grayMat = null;
        tempMat = null;
    }

    public Map<String, Mat> getImageCache() {
        return imageCache;
    }

    public Map<Integer, String> getFoundItemsMap() {
        return foundItemsMap;
    }

    @Override
    public void shutdownFactory() {
        api.shutdownFactory();
    }

    @Override
    public void reportRoundingCompletion() {
        api.reportRoundingCompletion();
    }

    @Override
    public void takeTargetItemSnapshot() {
        api.takeTargetItemSnapshot();
    }

    @Override
    public Kinematics getRobotKinematics() {
        return api.getRobotKinematics();
    }

    @Override
    public Result moveTo(Point point, Quaternion quaternion, boolean b) {
        return api.moveTo(point, quaternion, b);
    }

    @Override
    public void relativeMoveTo(Point point, Quaternion quaternion, boolean b) {
        api.relativeMoveTo(point, quaternion, b);
    }

    @Override
    public void setAreaInfo(int index, String info, int number) {
        api.setAreaInfo(index, info, number);
    }

    @Override
    public void flashlightControlFront(float power) {
        api.flashlightControlFront(power);
    }

    @Override
    public void flashlightControlBack(float power) {
        api.flashlightControlBack(power);
    }

    @Override
    public Mat getMatNavCam() {
        return api.getMatNavCam();
    }

    @Override
    public void saveMatImage(Mat image, String tag) {
        api.saveMatImage(image, tag);
    }

    @Override
    public void notifyRecognitionItem() {
        api.notifyRecognitionItem();
    }

    @Override
    public void internalStartMission() {
        api.startMission();
    }

    @Override
    public void internalHandleEmergency() {
        handleEmergencyStop();
    }

    @Override
    public void internalInitCamera() {
        initializeCameraParameters();
    }

    @Override
    public void internalHandleFailure() {
        handleMissionFailure();
    }

    @Override
    public void internalCleanup() {
        cleanup();
    }

    @Override
    public void cleanImageCache() {
        long currentTime = System.currentTimeMillis();
        imageCache.entrySet().removeIf(entry -> {
            if (imageCache.size() > MAX_CACHE_SIZE || 
                currentTime - lastDetectionTime.getOrDefault(entry.getKey().hashCode(), 0L) > MAX_CACHE_AGE) {
                Mat mat = entry.getValue();
                if (mat != null && !mat.empty()) {
                    mat.release();
                }
                return true;
            }
            return false;
        });
    }

    @Override
    public Mat acquireMatFromPool() {
        return MatPool.acquire();
    }

    @Override
    public void releaseMatToPool(Mat mat) {
        MatPool.release(mat);
    }

    @Override
    public void clearMatPool() {
        MatPool.clearPool();
    }

    @Override
    public int getMaxCacheSize() {
        return MAX_CACHE_SIZE;
    }

    @Override
    public long getMaxCacheAge() {
        return MAX_CACHE_AGE;
    }
}
