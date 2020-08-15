# ROS-Turtlebot-maze
Turtlebot maze solver that senses environment through laser scans and navigates. A mini Project from Robot Ignite Academy Course
Two classes has been implemented :
  1. /src/src/project.py - Includes the algorithm for navigation and obstacle detection.
  2. /src/src/robot_control_class.py - Contains sensor data acquisition and methods.
  
# Tools:
  python
  numpy
  ROS

# Demo

![20200120_190819](https://user-images.githubusercontent.com/47297221/72735002-f9dd2200-3bc0-11ea-8ba3-5ee3ab3c2c8a.gif)















### Results

![Image](https://github.com/aditya-167/2D-feature-tracking-CV/blob/master/images/ss.png)

Lidar and Camera TTC

![TopView](https://github.com/aditya-167/2D-feature-tracking-CV/blob/master/images/sw.png)

Top view lidar

## [Rubric](https://review.udacity.com/#!/rubrics/2550/view) Points
---

### FP.1 Match 3D Objects

**Criteria:**

* Implement the method "matchBoundingBoxes", which takes as input both the previous and the current data frames and provides as output the ids of the matched regions of interest (i.e. the boxID property). 
* Matches must be the ones with the highest number of keypoint correspondences.

* Solution: Function `matchBoundingBoxes` at the `camFusion_Student.cpp`
```c++
void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    int p = prevFrame.boundingBoxes.size();
    int c = currFrame.boundingBoxes.size();

    // store matched bounding boxes in array of size previous and current frame boundingbox size
    int match_box_cnts[p][c] = { };
    // loop through all matches
    for (auto it = matches.begin(); it != matches.end() - 1; ++it)     
    {
        //get query index from previous frame
        cv::KeyPoint query = prevFrame.keypoints[it->queryIdx];
        auto query_pt = cv::Point(query.pt.x, query.pt.y);
        bool query_found = false;
        // get matched index from previous frame to current frame called train index
        cv::KeyPoint train = currFrame.keypoints[it->trainIdx];
        auto train_pt = cv::Point(train.pt.x, train.pt.y);
        bool train_found = false;
        std::vector<int> query_id, train_id;

        //loop through all previous frames and check if that frames bounding box has queried index of keypoints and if found, push it in query_id vector
        for (int i = 0; i < p; i++) {
            if (prevFrame.boundingBoxes[i].roi.contains(query_pt))             
            {
                query_found = true;
                query_id.push_back(i);
            }
        }
        //loop through all current frames and check if that frames bounding box has train index of matched keypoints and if found, push it in train_id vector

        for (int i = 0; i < c; i++) {
            if (currFrame.boundingBoxes[i].roi.contains(train_pt))             
            {
                train_found= true;
                train_id.push_back(i);
            }
        }
        // if both query and matched train index found in bounding box roi of both previous and current frames, store it in matched bounding box array.
        if (query_found && train_found)
        {
            for (auto id_prev: query_id)
                for (auto id_curr: train_id)
                     match_box_cnts[id_prev][id_curr] += 1;
        }
    }
    //  get best match bounding boxes out of matched array
    for (int i = 0; i < p; i++)
    {
         int max_count = 0;
         int id_max = 0;
         for (int j = 0; j < c; j++)
             if (match_box_cnts[i][j] > max_count)
             {
                  max_count = match_box_cnts[i][j];
                  id_max = j;
             }
          bbBestMatches[i] = id_max;
    }
    bool bMsg = true;
    if (bMsg)
        for (int i = 0; i < p; i++)
             cout << "Box " << i << " matches " << bbBestMatches[i]<< " box" << endl;   
}

```

### FP.2 Compute Lidar-based TTC

**Criteria:**
* Compute the time-to-collision in second for all matched 3D objects using only Lidar measurements from the matched bounding boxes between current and previous frame.
#### (Answer):

* Solution: Function `computeTTCLidar` at the `camFusion_Student.cpp`
```c++
void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    double dT = 1 / frameRate;
    double laneWidth = 4.0; // assumed width of the ego lane
    vector<double> xPrev, xCurr;

    // find Lidar points within ego lane
    for (auto it = lidarPointsPrev.begin(); it != lidarPointsPrev.end(); ++it)
    {
        if (abs(it->y) <= laneWidth / 2.0)
        { // 3D point within ego lane?
            xPrev.push_back(it->x);
        }
    }
    for (auto it = lidarPointsCurr.begin(); it != lidarPointsCurr.end(); ++it)
    {
        if (abs(it->y) <= laneWidth / 2.0)
        { // 3D point within ego lane?
            xCurr.push_back(it->x);
        }
    }
    double minXPrev = 0;
    double minXCurr = 0;
    if (xPrev.size() > 0)
    {
       for (auto x: xPrev)
            minXPrev += x;
       minXPrev = minXPrev / xPrev.size();
    }
    if (xCurr.size() > 0)
    {
       for (auto x: xCurr)
           minXCurr += x;
       minXCurr = minXCurr / xCurr.size();
    }
    // compute TTC from both measurements
    cout << "minXCurr: " << minXCurr << endl;
    cout << "minXPrev: " << minXPrev << endl;
    TTC = minXCurr * dT / (minXPrev - minXCurr);
}
```

### FP.3 Associate Keypoint Correspondences with Bounding Boxes

**Criteria:**

* Prepare the TTC computation based on camera measurements by associating keypoint correspondences to the bounding boxes which enclose them. 
* All matches which satisfy this condition must be added to a vector in the respective bounding box.
#### (Answer):

* Solution: Function `clusterKptMatchesWithROI` at the `camFusion_Student.cpp`
```c++
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    double mean = 0;
    std::vector<cv::DMatch> kptMatches_roi;
    for (auto it = kptMatches.begin(); it != kptMatches.end(); ++it)
    {
        cv::KeyPoint kp = kptsCurr.at(it->trainIdx);
        if (boundingBox.roi.contains(cv::Point(kp.pt.x, kp.pt.y)))
            kptMatches_roi.push_back(*it);
    }
    for  (auto it = kptMatches_roi.begin(); it != kptMatches_roi.end(); ++it)
         mean += it->distance;
    cout << "Find " << kptMatches_roi.size()  << " matches" << endl;
    if (kptMatches_roi.size() > 0)
         mean = mean/kptMatches_roi.size();
    
    else return;
    double threshold = mean * 0.7;
    for  (auto it = kptMatches_roi.begin(); it != kptMatches_roi.end(); ++it)
    {
       if (it->distance < threshold)
           boundingBox.kptMatches.push_back(*it);
    }
    cout << "Removing " << boundingBox.kptMatches.size()  << " matches" << endl;
}
```

### FP.4 Compute Camera-based TTC

**Criteria:**

* Compute the time-to-collision in second for all matched 3D objects using only keypoint correspondences from the matched bounding boxes between current and previous frame.
#### (Answer):
* Solution: Function `computeTTCCamera` at the `camFusion_Student.cpp`
```c++
void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    double dT = 1 / frameRate;
    double laneWidth = 4.0; // assumed width of the ego lane
    vector<double> xPrev, xCurr;

    // find Lidar points within ego lane
    for (auto it = lidarPointsPrev.begin(); it != lidarPointsPrev.end(); ++it)
    {
        if (abs(it->y) <= laneWidth / 2.0)
        { // 3D point within ego lane?
            xPrev.push_back(it->x);
        }
    }
    for (auto it = lidarPointsCurr.begin(); it != lidarPointsCurr.end(); ++it)
    {
        if (abs(it->y) <= laneWidth / 2.0)
        { // 3D point within ego lane?
            xCurr.push_back(it->x);
        }
    }
    double minXPrev = 0;
    double minXCurr = 0;
    if (xPrev.size() > 0)
    {
       for (auto x: xPrev)
            minXPrev += x;
       minXPrev = minXPrev / xPrev.size();
    }
    if (xCurr.size() > 0)
    {
       for (auto x: xCurr)
           minXCurr += x;
       minXCurr = minXCurr / xCurr.size();
    }
    // compute TTC from both measurements
    cout << "minXCurr: " << minXCurr << endl;
    cout << "minXPrev: " << minXPrev << endl;
    TTC = minXCurr * dT / (minXPrev - minXCurr);
}
```

### FP.5 Performance Evaluation 1

**Criteria**:

* Find examples where the TTC estimate of the Lidar sensor does not seem plausible. 
* Describe your observations and provide a sound argumentation why you think this happened.
#### (Answer):
* Similar to 2-D feature extraction project, I created a loop to test all possible combinations of detectors and descriptors.
  * detectors: `SHITOMASI`, `FAST`, `BRISK`, `ORB`, `AKAZE`
  * descriptors: `BRISK`, `BRIEF`, `ORB`, `FREAK`  
  * Saved Performance Results of All Combination: please check `Evaluation/evaluation_2.csv`.
  * Saved Results Images of All Combination: please check `Evaluation/ImagesResult/` folder.
* Bounding box contains some points which is not involved front car.
* Lidar is not a 100% perfect sensor. This can be also effected by sun light or reflection.
* Lidar-Camera Calibration can be one of the reason. If calibration has some error, bounding box can include wrong points.


* In case of 14-18 frames, by the eye, the distance of the front vehicle decreased, but the TTC increased.
* TTC from Lidar is not correct because of some outliers and some unstable points from preceding vehicle's front mirrors, those need to be filtered out.
* Some examples with wrong TTC estimate of the Lidar sensor:

Frame Number        | IMAGE               
--------------------| -------------------
14                  |![alt text](https://github.com/studian/SFND_P4_3D_Object_Tracking/blob/master/resultsImages/SHITOMASI_FREAK/0000000014.png) 
15                  |![alt text](https://github.com/studian/SFND_P4_3D_Object_Tracking/blob/master/resultsImages/SHITOMASI_FREAK/0000000015.png)
16                  |![alt text](https://github.com/studian/SFND_P4_3D_Object_Tracking/blob/master/resultsImages/SHITOMASI_FREAK/0000000016.png) 
17                  |![alt text](https://github.com/studian/SFND_P4_3D_Object_Tracking/blob/master/resultsImages/SHITOMASI_FREAK/0000000017.png) 
18                  |![alt text](https://github.com/studian/SFND_P4_3D_Object_Tracking/blob/master/resultsImages/SHITOMASI_FREAK/0000000018.png) 

* TTC from Lidar is not correct because of Lidar points from preceding vehicle front mirrors. 
* Need to delete Lidar points from preceding vehicle front mirrors.
 

### FP.6 Performance Evaluation 2
* Run several detector / descriptor combinations and look at the differences in TTC estimation. 
* Find out which methods perform best and also include several examples where camera-based TTC estimation is way off. 
* As with Lidar, describe your observations again and also look into potential reasons.
#### (Answer):
* I created a loop in code to test all possible combinations of detectors and descriptors and saved the results.
  * detectors: `SHITOMASI`, `FAST`, `BRISK`, `ORB`, `AKAZE`
  * descriptors: `BRISK`, `BRIEF`, `ORB`, `FREAK`  
  * Saved Results of All Combination: please check `FP_6_Performance_Evaluation_2.csv` file [CSV file](https://github.com/studian/SFND_P4_3D_Object_Tracking/FP_6_Performance_Evaluation_2.csv).
* Analysis of All Combination: please check `FP_6_Performance_Evaluation_2_analysis.xlsx` file [Excel file](https://github.com/studian/SFND_P4_3D_Object_Tracking/FP_6_Performance_Evaluation_2_analysis.xlsx).
* Certain detector/descriptor combinations, especially the `ORB` detectors, produced very unreliable camera TTC estimates.
* The TOP3 detector / descriptor combinations as the best choice for our purpose of detecting keypoints on vehicles are: 
  * `SHITOMASI` / `FREAK`
  * `AKAZE` / `BRISK`
  * `AKAZE` / `FREAK`







