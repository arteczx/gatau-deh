package jp.jaxa.iss.kibo.rpc.flyoverspace;

import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import gov.nasa.arc.astrobee.Result;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.imgproc.Imgproc;
import org.opencv.aruco.Aruco;
import org.opencv.aruco.Dictionary;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.HashSet;
import java.util.ArrayList;
import java.util.PriorityQueue;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.TimeoutException;
import java.util.Objects;
import java.util.HashMap;
import java.util.Collections;
import gov.nasa.arc.astrobee.Kinematics;

public class Helper {
    public static final double MAX_VELOCITY = 0.5;  // m/s
    public static final double MIN_DISTANCE = 0.05; // m
    public static final double MIN_ANGLE = 7.5;     // degrees
    public static final double MAX_THRUST_X = 0.6;  // N
    public static final double ROBOT_MASS = 10.0;   // kg

    public static final double[] KIZ1_BOUNDS = {10.3, -10.2, 4.32, 11.55, -6.0, 5.57};
    public static final double[] KIZ2_BOUNDS = {9.5, -10.5, 4.02, 10.5, -9.6, 4.8};

    //x_min, y_min, z_min, x_max, y_max, z_max
    public static final double[][] KOZ_BOUNDS = {
        //KOZ1
        {10.87, -9.5, 4.27, 11.6, -9.45, 4.97},    //pos 1
        {10.25, -9.5, 4.97, 10.87, -9.45, 5.62},   //pos 2
        //KOZ2
        {10.87, -8.5, 4.97, 11.6, -8.45, 5.62},    //pos 1
        {10.25, -8.5, 4.27, 10.7, -8.45, 4.97},    //pos 2
        //KOZ3
        {10.87, -7.40, 4.27, 11.6, -7.35, 4.97},   //pos 1
        {10.25, -7.40, 4.97, 10.87, -7.35, 5.62}   //pos 2
    };

    // Safe waypoints for navigation
    public static final Point[] SAFE_WAYPOINTS = {
        new Point(10.0, -9.8, 4.5),   // near dock
        new Point(10.3, -9.0, 4.5),   //between KOZ1 and KOZ2
        new Point(10.3, -8.0, 4.5),   //between KOZ2 and KOZ3
        new Point(10.3, -7.0, 4.5)    // past KOZ3
    };

    public static final Point[] AREA_POINTS = new Point[] {
        new Point(10.95, -10.585, 4.82), //area 1 center
        new Point(10.925, -8.875, 3.76203), //area 2 center
        new Point(10.925, -7.875, 3.76203), //area 3 center
        new Point(9.866984, -6.8525, 4.32)  //area 4 center
    };

    public static final Point START_POINT = new Point(9.815, -9.806, 4.293);
    public static final Point ASTRONAUT_POINT = new Point(11.143, -6.7607, 4.9654);
    public static final Quaternion START_QUATERNION = new Quaternion(1f, 0f, 0f, 0f);
    public static final Quaternion ASTRONAUT_QUATERNION = new Quaternion(0f, 0f, 0.707f, 0.707f);

    public static final double[] NAV_CAM_OFFSET = {0.1177, -0.0422, -0.0826};
    public static final double[] HAZ_CAM_OFFSET = {0.1328, 0.0362, -0.0826};

    public static final long DETECTION_COOLDOWN = 500; //ms

    public static final Map<Integer, String> MARKER_TO_ITEM;
    static {
        Map<Integer, String> map = new HashMap<>();
        map.put(1, "tape");
        map.put(2, "top");
        map.put(3, "screwdriver");
        map.put(4, "beaker");
        map.put(5, "hammer");
        map.put(6, "pipette");
        map.put(7, "wrench");
        map.put(8, "thermometer");
        map.put(9, "watch");
        map.put(10, "goggle");
        MARKER_TO_ITEM = Collections.unmodifiableMap(map);
    }

    public static class Node implements Comparable<Node> {
        Point point;
        Node parent;
        double gCost;
        double hCost;
        
        public Node(Point point, Node parent, double gCost, double hCost) {
            this.point = point;
            this.parent = parent;
            this.gCost = gCost;
            this.hCost = hCost;
        }
        
        public double getFCost() {
            return gCost + hCost;
        }
        
        @Override
        public int compareTo(Node other) {
            return Double.compare(this.getFCost(), other.getFCost());
        }
        
        @Override
        public boolean equals(Object obj) {
            if (obj instanceof Node) {
                Node other = (Node) obj;
                return point.getX() == other.point.getX() &&
                       point.getY() == other.point.getY() &&
                       point.getZ() == other.point.getZ();
            }
            return false;
        }
        
        @Override
        public int hashCode() {
            return Objects.hash(point.getX(), point.getY(), point.getZ());
        }
    }

    public static void initializeCameraParameters(Mat cameraMatrix, Mat distCoeffs, double[][] camParams) {
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                cameraMatrix.put(i, j, camParams[0][i * 3 + j]);
            }
        }
        for (int i = 0; i < 5; i++) {
            distCoeffs.put(0, i, camParams[1][i]);
        }
    }

    public static double distance(Point p1, Point p2) {
        double dx = p1.getX() - p2.getX();
        double dy = p1.getY() - p2.getY();
        double dz = p1.getZ() - p2.getZ();
        return Math.sqrt(dx*dx + dy*dy + dz*dz);
    }

    public static boolean isPointInBounds(Point p, double[] bounds) {
        return p.getX() >= bounds[0] && p.getX() <= bounds[3] &&
               p.getY() >= bounds[1] && p.getY() <= bounds[4] &&
               p.getZ() >= bounds[2] && p.getZ() <= bounds[5];
    }

    public static boolean isPointSafe(Point p) {
        boolean inKiz = isPointInBounds(p, KIZ1_BOUNDS) || isPointInBounds(p, KIZ2_BOUNDS);
        if (!inKiz) return false;

        for (double[] koz : KOZ_BOUNDS) {
            if (isPointInBounds(p, koz)) return false;
        }
        return true;
    }

    public static boolean isValidDistance(Point p1, Point p2) {
        double dist = distance(p1, p2);
        return dist >= MIN_DISTANCE;
    }

    public static boolean isValidRotation(Quaternion q1, Quaternion q2) {
        double dotProduct = q1.getX()*q2.getX() + q1.getY()*q2.getY() + 
                          q1.getZ()*q2.getZ() + q1.getW()*q2.getW();
        double angle = Math.toDegrees(2 * Math.acos(Math.abs(dotProduct)));
        return angle >= MIN_ANGLE;
    }

    public static boolean isPathSafe(Point start, Point end) {
        if (!isValidDistance(start, end)) {
            return false;
        }

        double distance = distance(start, end);
        double timeRequired = distance / MAX_VELOCITY;
        double acceleration = (2 * distance) / (timeRequired * timeRequired);
        double force = ROBOT_MASS * acceleration;
        
        if (force > MAX_THRUST_X) {
            return false;
        }

        int steps = Math.max(20, (int)(distance * 50)); 
        
        for (int i = 0; i <= steps; i++) {
            double t = i / (double) steps;
            Point p = new Point(
                start.getX() + t * (end.getX() - start.getX()),
                start.getY() + t * (end.getY() - start.getY()),
                start.getZ() + t * (end.getZ() - start.getZ())
            );
            if (!isPointSafe(p)) return false;
        }
        return true;
    }

    public static Point getCorrectedPointForNavCam(Point original) {
        return new Point(
            original.getX() + NAV_CAM_OFFSET[0],
            original.getY() + NAV_CAM_OFFSET[1],
            original.getZ() + NAV_CAM_OFFSET[2]
        );
    }

    public static String identifyItem(List<Integer> markers) {
        if (markers == null || markers.isEmpty()) {
            return "unknown";
        }
        
        for (Integer markerId : markers) {
            String item = MARKER_TO_ITEM.get(markerId);
            if (item != null) {
                return item;
            }
        }
        return "unknown";
    }

    public static boolean verifyImageQuality(Mat image) {
        Mat gray = new Mat();
        Imgproc.cvtColor(image, gray, Imgproc.COLOR_BGR2GRAY);
        Scalar mean = Core.mean(gray);
        gray.release();
        
        if (mean.val[0] < 50 || mean.val[0] > 200) {
            return false;
        }
        
        Mat laplacian = new Mat();
        Imgproc.Laplacian(image, laplacian, CvType.CV_64F);
        MatOfDouble median = new MatOfDouble();
        MatOfDouble std = new MatOfDouble();
        Core.meanStdDev(laplacian, median, std);
        double variance = Math.pow(std.get(0,0)[0], 2);
        laplacian.release();
        
        return variance > 100;
    }

    public static List<Point> findPathAStar(Point start, Point goal, ApiService api) {
        try {
            PriorityQueue<Node> openSet = new PriorityQueue<>();
            Set<String> closedSet = new HashSet<>();
            int maxIterations = 1000;
            
            Node startNode = new Node(start, null, 0, heuristic(start, goal));
            openSet.add(startNode);
            
            while (!openSet.isEmpty() && maxIterations-- > 0) {
                Node current = openSet.poll();
                String currentKey = pointToKey(current.point);
                
                if (isCloseEnough(current.point, goal)) {
                    return smoothPath(reconstructPath(current));
                }
                
                if (closedSet.contains(currentKey)) continue;
                closedSet.add(currentKey);
                
                List<Point> neighbors = getOptimizedNeighbors(current.point, goal);
                
                for (Point neighbor : neighbors) {
                    String neighborKey = pointToKey(neighbor);
                    if (closedSet.contains(neighborKey)) continue;
                    
                    double newCost = current.gCost + distance(current.point, neighbor);
                    double hCost = heuristic(neighbor, goal) * 1.1;
                    
                    Node neighborNode = new Node(neighbor, current, newCost, hCost);
                    
                    Result moveTest = api.moveTo(neighbor, 
                        api.getRobotKinematics().getOrientation(), false);
                    
                    if (moveTest != null && isPointSafe(neighbor)) {
                        openSet.add(neighborNode);
                    }
                }
            }
            
            if (isPathSafe(start, goal)) {
                List<Point> directPath = new ArrayList<>();
                directPath.add(start);
                directPath.add(goal);
                return directPath;
            }
            
            return null;
        } catch (Exception e) {
            throw new RuntimeException("Path finding failed", e);
        }
    }

    private static List<Point> getOptimizedNeighbors(Point current, Point goal) {
        List<Point> neighbors = new ArrayList<>();
        double step = 0.2; //20cm steps
        
        double dx = goal.getX() - current.getX();
        double dy = goal.getY() - current.getY();
        double dz = goal.getZ() - current.getZ();
        
        //normalize
        double length = Math.sqrt(dx*dx + dy*dy + dz*dz);
        if (length > 0) {
            dx /= length;
            dy /= length;
            dz /= length;
        }
        
        for (int i = -1; i <= 1; i++) {
            for (int j = -1; j <= 1; j++) {
                for (int k = -1; k <= 1; k++) {
                    if (i == 0 && j == 0 && k == 0) continue;
                    
                    double weight = (i*dx + j*dy + k*dz + 1) / 2;
                    if (weight > 0.3) {
                        Point neighbor = new Point(
                            current.getX() + i * step,
                            current.getY() + j * step,
                            current.getZ() + k * step
                        );
                        neighbors.add(neighbor);
                    }
                }
            }
        }
        
        return neighbors;
    }

    private static String pointToKey(Point p) {
        return String.format("%.2f,%.2f,%.2f", p.getX(), p.getY(), p.getZ());
    }

    private static boolean isCloseEnough(Point p1, Point p2) {
        return distance(p1, p2) < 0.05; //5cm threshold
    }

    private static double heuristic(Point start, Point goal) {
        return distance(start, goal);
    }

    private static List<Point> reconstructPath(Node endNode) {
        List<Point> path = new ArrayList<>();
        Node current = endNode;
        
        while (current != null) {
            path.add(0, current.point);
            current = current.parent;
        }
        
        return smoothPath(path);
    }

    private static List<Point> smoothPath(List<Point> path) {
        if (path.size() <= 2) return path;
        
        List<Point> smoothed = new ArrayList<>();
        smoothed.add(path.get(0));
        
        int i = 0;
        while (i < path.size() - 1) {
            int j = path.size() - 1;
            while (j > i) {
                if (isPathSafe(path.get(i), path.get(j)) && 
                    isValidDistance(path.get(i), path.get(j)) &&
                    isPointSafe(path.get(j))) {
                    smoothed.add(path.get(j));
                    i = j;
                    break;
                }
                j--;
            }
            if (j == i) {
                i++;
                if (i < path.size()) {
                    smoothed.add(path.get(i));
                }
            }
        }
        
        return smoothed;
    }

    public static List<Integer> detectArUcoMarkers(Mat image, Dictionary dictionary, Mat grayMat, Mat tempMat, List<Mat> corners) {
        Mat ids = new Mat();
        corners.clear(); //reuse list instead of creating new
        
        try {
            if (image.width() > 640) {
                if (tempMat == null) tempMat = new Mat();
                Imgproc.resize(image, tempMat, new Size(640, 480));
                image = tempMat;
            }
            
            Imgproc.cvtColor(image, grayMat, Imgproc.COLOR_BGR2GRAY);
            Imgproc.GaussianBlur(grayMat, grayMat, new Size(5, 5), 0);
            
            Aruco.detectMarkers(grayMat, dictionary, corners, ids);
            
            List<Integer> detectedMarkers = new ArrayList<>(ids.rows());
            if (!ids.empty()) {
                for (int i = 0; i < ids.rows(); i++) {
                    detectedMarkers.add((int) ids.get(i, 0)[0]);
                }
            }
            return detectedMarkers;
            
        } finally {
            ids.release(); //release temporary Mat
        }
    }

    public static void releaseMatObjects(Mat grayMat, Mat tempMat, List<Mat> corners) {
        try {
            if (grayMat != null) {
                grayMat.release();
            }
            if (tempMat != null) {
                tempMat.release();
            }
            corners.forEach(Mat::release);
            corners.clear();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public static void handleEmergencyStop(ApiService service, Point currentPos, Quaternion currentQuat) {
        try {
            Point safePoint = findNearestSafePoint(currentPos);
            
            Result moveResult = service.moveTo(safePoint, currentQuat, true);
            
            if (moveResult == null) {
                Point relativeMove = new Point(
                    safePoint.getX() - currentPos.getX(),
                    safePoint.getY() - currentPos.getY(),
                    safePoint.getZ() - currentPos.getZ()
                );
                service.relativeMoveTo(relativeMove, currentQuat, true);
            }
            
            service.setAreaInfo(999, "EMERGENCY_STOP", 1); 
            
        } catch (Exception e) {
            e.printStackTrace();
            service.setAreaInfo(999, "ERROR: " + e.getMessage(), 0);
        }
    }

    public static void cleanup(ApiService service, ExecutorService executor, 
                             Map<String, Mat> imageCache, Mat cameraMatrix, 
                             Mat distCoeffs) {
        try {
            shutdownLights(service);
            releaseImageResources(imageCache, cameraMatrix, distCoeffs);
            shutdownExecutor(executor);
        } catch (Exception e) {
            e.printStackTrace();
        } finally {
            try {
                service.shutdownFactory();
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }

    private static void shutdownLights(ApiService service) {
        try {
            service.flashlightControlFront(0f);
            service.flashlightControlBack(0f);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    private static void releaseImageResources(Map<String, Mat> imageCache, 
                                            Mat cameraMatrix, Mat distCoeffs) {
        try {
            if (cameraMatrix != null) {
                cameraMatrix.release();
            }
            if (distCoeffs != null) {
                distCoeffs.release();
            }
            
            if (imageCache != null) {
                imageCache.values().forEach(mat -> {
                    if (mat != null && !mat.empty()) {
                        mat.release();
                    }
                });
                imageCache.clear();
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    private static void shutdownExecutor(ExecutorService executor) {
        if (executor != null && !executor.isShutdown()) {
            try {
                executor.shutdown();
                if (!executor.awaitTermination(5, TimeUnit.SECONDS)) {
                    executor.shutdownNow();
                }
            } catch (InterruptedException e) {
                executor.shutdownNow();
                Thread.currentThread().interrupt();
            }
        }
    }

    private static Point findNearestSafePoint(Point currentPos) {
        Point nearest = SAFE_WAYPOINTS[0];
        double minDist = Double.MAX_VALUE;
        
        for (Point waypoint : SAFE_WAYPOINTS) {
            double dist = distance(waypoint, currentPos);
            if (dist < minDist && isPointSafe(waypoint)) {
                minDist = dist;
                nearest = waypoint;
            }
        }
        return nearest;
    }

    public static Quaternion createScanningQuaternion(Point target, Point startPoint) {
        //calc direction to face the target
        float yaw = (float) Math.atan2(target.getY() - startPoint.getY(), 
                                     target.getX() - startPoint.getX());
        return new Quaternion(0f, 0f, (float) Math.sin(yaw/2), (float) Math.cos(yaw/2));
    }

    public static Quaternion adjustQuaternionByDegrees(Quaternion base, float degrees) {
        float radians = (float) Math.toRadians(degrees);
        float sin = (float) Math.sin(radians/2);
        float cos = (float) Math.cos(radians/2);
        return new Quaternion(
            base.getX()*cos - base.getY()*sin,
            base.getX()*sin + base.getY()*cos,
            base.getZ(),
            base.getW()
        );
    }

    public static Quaternion calculateIntermediateQuaternion(
            Quaternion start, 
            Quaternion end, 
            Point currentPos, 
            Point targetPos) {
        
        //calculate progress along path
        double totalDistance = distance(currentPos, targetPos);
        double progress = Math.min(1.0, distance(currentPos, targetPos) / totalDistance);
        
        //interpolate quaternion
        return new Quaternion(
            (float)(start.getX() + (end.getX() - start.getX()) * progress),
            (float)(start.getY() + (end.getY() - start.getY()) * progress),
            (float)(start.getZ() + (end.getZ() - start.getZ()) * progress),
            (float)(start.getW() + (end.getW() - start.getW()) * progress)
        );
    }

    public static void completeTargetOperationOptimized(
            ApiService service,
            Point astronautPoint,
            Quaternion astronautQuaternion,
            int targetAreaId,
            Point[] areaPoints,
            ApiService service2) {
        
        String targetItem = "unknown";
        Point correctedAstronautPoint = getCorrectedPointForNavCam(astronautPoint);
        
        service.moveToWithRetry(correctedAstronautPoint, astronautQuaternion);
        
        for (int attempt = 0; attempt < 5 && "unknown".equals(targetItem); attempt++) {
            Mat astronautImage = service.getMatNavCam();
            List<Integer> targetMarkers = service.detectArUcoMarkers(astronautImage, 
                String.format("astronaut_target_attempt_%d", attempt));
            targetItem = identifyItem(targetMarkers);
            
            if (!"unknown".equals(targetItem)) {
                service.saveMatImage(astronautImage, "target_item_detected_" + attempt);
                break;
            }
        }
        
        service.notifyRecognitionItem();
        
        if (targetAreaId != -1) {
            Point targetPoint = areaPoints[targetAreaId - 1];
            Point correctedTargetPoint = getCorrectedPointForNavCam(targetPoint);
            
            if (isPointSafe(targetPoint)) {
                service.moveToWithRetry(correctedTargetPoint, 
                    service.createScanningQuaternion(targetPoint));
                Mat finalImage = service.getMatNavCam();
                service.saveMatImage(finalImage, "final_target");
                service.takeTargetItemSnapshot();
                service.reportRoundingCompletion();
            }
        }
    }

    public static void moveToWithRetry(ApiService service, Point targetPoint, 
                                     Quaternion targetQuaternion) {
        try {
            double MAX_SAFE_DISTANCE = 2.0; //meters
            
            if (distance(service.getRobotKinematics().getPosition(), targetPoint) > MAX_SAFE_DISTANCE) {
                handleEmergencyStop(service, service.getRobotKinematics().getPosition(), 
                                  service.getRobotKinematics().getOrientation());
                throw new RuntimeException("Movement distance exceeds safety limit");
            }
            
            int maxRetries = 3;
            int retryCount = 0;
            boolean success = false;
            
            Point currentPosition = service.getRobotKinematics().getPosition();
            Quaternion currentQuaternion = service.getRobotKinematics().getOrientation();
            
            if (!isValidDistance(currentPosition, targetPoint) || 
                !isValidRotation(currentQuaternion, targetQuaternion)) {
                handleEmergencyStop(service, currentPosition, currentQuaternion);
                throw new RuntimeException("Invalid movement parameters");
            }
            
            while (!success && retryCount < maxRetries) {
                try {
                    List<Point> path = findPathAStar(currentPosition, targetPoint, service);
                    
                    if (path == null || path.isEmpty()) {
                        throw new RuntimeException("No valid path found");
                    }
                    
                    for (Point waypoint : path) {
                        if (!isValidDistance(currentPosition, waypoint)) {
                            throw new RuntimeException("Invalid distance to waypoint");
                        }

                        Quaternion intermediateQuat = calculateIntermediateQuaternion(
                            currentQuaternion, 
                            targetQuaternion, 
                            currentPosition, 
                            waypoint
                        );

                        if (!isValidRotation(currentQuaternion, intermediateQuat)) {
                            throw new RuntimeException("Invalid rotation to waypoint");
                        }
                        
                        //execute movement with all safety checks
                        Result moveResult = service.moveTo(waypoint, intermediateQuat, true);
                        if (moveResult == null) {
                            throw new RuntimeException("Movement failed");
                        }
                        
                        double distance = distance(currentPosition, waypoint);
                        double moveTime = distance / MAX_VELOCITY;
                        Thread.sleep((long)(moveTime * 1000) + 1000); 
                        
                        currentPosition = waypoint;
                        currentQuaternion = intermediateQuat;
                    }
                    
                    //final movement to target with exact quaternion
                    if (!isValidRotation(currentQuaternion, targetQuaternion)) {
                        service.moveTo(currentPosition, targetQuaternion, true);
                        Thread.sleep(1000); //final rotation
                    }
                    
                    success = true;
                    
                } catch (Exception e) {
                    retryCount++;
                    if (retryCount >= maxRetries) {
                        handleEmergencyStop(service, currentPosition, currentQuaternion);
                        throw new RuntimeException("Failed to move after " + maxRetries + " attempts", e);
                    }
                    Thread.sleep(1000);
                }
            }
        } catch (Exception e) {
            handleEmergencyStop(service, service.getRobotKinematics().getPosition(), 
                              service.getRobotKinematics().getOrientation());
            throw new RuntimeException("Movement failed with emergency", e);
        }
    }

    public static void handleMissionFailure(ApiService service) {
        Point currentPos = service.getRobotKinematics().getPosition();
        Quaternion currentQuat = service.getRobotKinematics().getOrientation();
        Point safePoint = findNearestSafePoint(currentPos);
        moveToWithRetry(service, safePoint, currentQuat);
    }

    public static boolean scanAreaOptimized(ApiService service, int areaIndex, 
                                          Point[] areaPoints, ExecutorService executor,
                                          ApiService service2) {
        try {
            service.flashlightControlFront(0.05f);
            
            float[] angles = calculateOptimalScanAngles(areaPoints[areaIndex], START_POINT);
            
            for (float angle : angles) {
                Quaternion rotatedQuat = adjustQuaternionByDegrees(
                    service.createOptimizedScanningQuaternion(areaPoints[areaIndex]), 
                    angle
                );
                
                if (!service.isValidMovement(areaPoints[areaIndex], rotatedQuat)) {
                    continue;
                }

                try {
                    CompletableFuture<Boolean> detectionFuture = CompletableFuture.supplyAsync(() -> {
                        return processImagesOptimized(service, areaIndex, service);
                    }, executor);

                    if (detectionFuture.get(1, TimeUnit.SECONDS)) {
                        Mat image = service.getMatNavCam();
                        if (verifyImageQuality(image)) {
                            service.saveMatImage(image, String.format("area_%d_item_found", areaIndex + 1));
                            return true;
                        } else {
                            service.flashlightControlFront(0.07f);
                            image = service.getMatNavCam();
                            service.saveMatImage(image, String.format("area_%d_item_found", areaIndex + 1));
                            service.flashlightControlFront(0.05f);
                            return true;
                        }
                    }
                } catch (TimeoutException te) {
                    continue;
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }
            return false;
            
        } finally {
            service.flashlightControlFront(0f);
        }
    }

    private static float[] calculateOptimalScanAngles(Point target, Point startPoint) {
        double baseAngle = Math.toDegrees(Math.atan2(
            target.getY() - startPoint.getY(),
            target.getX() - startPoint.getX()
        ));
        
        return new float[]{
            0f,
            (float)(baseAngle - 15),
            (float)(baseAngle + 15),
            (float)(baseAngle - 30),
            (float)(baseAngle + 30)
        };
    }

    private static boolean processImagesOptimized(ApiService service, int areaIndex, 
                                                ApiService service2) {
        Mat image = service.getMatNavCam();
        String cacheKey = "area_" + areaIndex;
        
        Mat gray = service.getImageCache().computeIfAbsent(cacheKey, k -> new Mat());
        Imgproc.cvtColor(image, gray, Imgproc.COLOR_BGR2GRAY);
        Imgproc.GaussianBlur(gray, gray, new Size(5, 5), 0);
        Imgproc.threshold(gray, gray, 127, 255, Imgproc.THRESH_BINARY);

        List<Integer> markers = service.detectArUcoMarkers(gray, "area_" + areaIndex);
        
        if (!markers.isEmpty()) {
            String detectedItem = identifyItem(markers);
            if (!"unknown".equals(detectedItem)) {
                service.setAreaInfo(areaIndex + 1, detectedItem, 1);
                service.getFoundItemsMap().put(areaIndex + 1, detectedItem);
                return true;
            }
        }
        return false;
    }

    public static void saveDebugImage(ApiService service, Mat image, String tag, int attempt) {
        service.saveMatImage(image, tag + "_" + attempt);
    }

    public static Point optimizeTargetPoint(Point originalTarget, ApiService service) {
        double optimalDistance = 0.3; //optimal distance for photo angle scoring
        Point currentPos = service.getRobotKinematics().getPosition();
        
        //calc vector from current position to target
        double dx = originalTarget.getX() - currentPos.getX();
        double dy = originalTarget.getY() - currentPos.getY();
        double dz = originalTarget.getZ() - currentPos.getZ();
        
        //normalize and adjust for optimal distance
        double length = Math.sqrt(dx*dx + dy*dy + dz*dz);
        if (length > 0) {
            dx = dx/length * optimalDistance;
            dy = dy/length * optimalDistance;
            dz = dz/length * optimalDistance;
        }
        
        //create optimized point
        Point optimizedPoint = new Point(
            originalTarget.getX() - dx,
            originalTarget.getY() - dy,
            originalTarget.getZ() - dz
        );
        
        return isPointSafe(optimizedPoint) ? optimizedPoint : findNearestSafePoint(optimizedPoint);
    }

    public static Quaternion createOptimizedScanningQuaternion(Point target, ApiService service) {
        Point currentPos = service.getRobotKinematics().getPosition();
        
        double dx = target.getX() - currentPos.getX();
        double dy = target.getY() - currentPos.getY();
        double dz = target.getZ() - currentPos.getZ();
        
        float yaw = (float) Math.atan2(dy, dx);
        
        float pitch = (float) Math.atan2(dz, Math.sqrt(dx*dx + dy*dy));
        pitch = Math.min(pitch, (float)Math.toRadians(30));
        
        float cy = (float) Math.cos(yaw * 0.5);
        float sy = (float) Math.sin(yaw * 0.5);
        float cp = (float) Math.cos(pitch * 0.5);
        float sp = (float) Math.sin(pitch * 0.5);
        
        return new Quaternion(
            sp * cy,
            cp * sy,
            sp * sy,
            cp * cy
        );
    }

    public static boolean isValidMovement(Point point, Quaternion quat, ApiService service) {
        return isPointSafe(point) && 
               isValidRotation(service.getRobotKinematics().getOrientation(), quat);
    }

    public static void executeMainMission(ApiService service, Point startPoint, 
                                        Quaternion startQuaternion, Point[] areaPoints,
                                        ExecutorService executor) {
        try {
            service.internalStartMission();

            CompletableFuture<Void> navigationFuture = CompletableFuture.runAsync(() -> {
                try {
                    Kinematics currentKinematics = service.getRobotKinematics();
                    Point currentPos = currentKinematics.getPosition();
                    if (distance(currentPos, startPoint) > 0.3) { 
                        moveToWithRetry(service, startPoint, startQuaternion);
                    }
                } catch (Exception e) {
                    service.internalHandleEmergency();
                    throw e;
                }
            }, executor);

            CompletableFuture<Void> detectionFuture = CompletableFuture.runAsync(() -> {
                service.internalInitCamera();
                Mat warmupImage = service.getMatNavCam();
                service.detectArUcoMarkers(warmupImage, "warmup");
                warmupImage.release();
            }, executor);

            try {
                navigationFuture.get(20, TimeUnit.SECONDS);
                detectionFuture.get(5, TimeUnit.SECONDS);
            } catch (TimeoutException e) {
                service.internalHandleEmergency();
                throw new RuntimeException("Mission initialization timeout", e);
            }

            for (int i = 0; i < areaPoints.length; i++) {
                final int areaIndex = i;
                
                try {
                    CompletableFuture<Void> movementFuture = CompletableFuture.runAsync(() -> {
                        try {
                            Point targetPoint = optimizeTargetPoint(areaPoints[areaIndex], service);
                            moveToWithRetry(service, targetPoint, 
                                service.createOptimizedScanningQuaternion(targetPoint));
                        } catch (Exception e) {
                            service.internalHandleEmergency();
                            throw e;
                        }
                    }, executor);

                    CompletableFuture<Boolean> processingFuture = CompletableFuture.supplyAsync(() -> {
                        return scanAreaOptimized(service, areaIndex, areaPoints, executor, service);
                    }, executor);

                    movementFuture.get(30, TimeUnit.SECONDS);
                    if (processingFuture.get(10, TimeUnit.SECONDS)) {
                        continue;
                    }
                } catch (Exception e) {
                    service.internalHandleEmergency();
                    throw new RuntimeException("Area scanning failed: " + areaIndex, e);
                }
            }

            completeTargetOperationOptimized(service, ASTRONAUT_POINT, ASTRONAUT_QUATERNION, 
                                           -1, areaPoints, service);
            
            service.reportRoundingCompletion();

        } catch (Exception e) {
            e.printStackTrace();
            service.internalHandleFailure();
        } finally {
            service.internalCleanup();
        }
    }
}
