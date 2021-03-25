#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include "rover_msgs/TargetPosition.hpp"
#include "rover_msgs/TargetPositionList.hpp"
#include <unistd.h>
#include "perception.hpp"

using namespace std;

int main() {
  /* --- Camera Initializations --- */
  Camera cam;
  cam.grab();

  #if PERCEPTION_DEBUG
    namedWindow("depth", 2);
  #endif
  
  #if AR_DETECTION
  Mat rgb;
  Mat src = cam.image();
  #endif

  /* -- LCM Messages Initializations -- */
  lcm::LCM lcm_;
  rover_msgs::TargetPositionList arTagsMessage;
  rover_msgs::TargetPosition* arTags = arTagsMessage.target_list;
  arTags[0].z = -1;
  arTags[1].z = -1;
  vector<int> buffer;
  buffer.push_back(0);
  buffer.push_back(0);

  /* --- AR Tag Initializations --- */
  TagDetector detector;
  pair<Tag, Tag> tagPair;
  
   /* --- AR Recording Initializations and Implementation--- */ 
  
  time_t now = time(0);
  char* ltm = ctime(&now);
  string timeStamp(ltm);

  #if AR_RECORD
  //initializing ar tag videostream object
  cam.record_ar_init();
  #endif

/* --- Main Processing Stuff --- */
  while (true) {
    //Check to see if we were able to grab the frame
    if (!cam.grab()) break;

    #if AR_DETECTION
    //Grab initial images from cameras
    Mat rgb;
    Mat src = cam.image();
    Mat depth_img = cam.depth();
    #endif

/* --- AR Tag Processing --- */
    //Set default values
    for(int i = 0; i < 3; i++) {
        arTags[i].z = -1;
        arTags[i].target_id = -1;
    }
    arTagsMessage.num_targets = 0;

    #if AR_DETECTION
      cerr<<"Finding AR Tags"<<endl;
      tagPair = detector.findARTags(src, depth_img, rgb);
      #if AR_RECORD
        cam.record_ar(rgb);
      #endif


      detector.updateDetectedTagInfo(arTags, tagPair, depth_img, src, rgb, buffer);

        //update num targets
    for(int i = 0; i < 3; i++) {
        if(arTags[i].target_id != -1) arTagsMessage.num_targets++;
        cerr << arTags[i].target_id << endl;
      }

    #if PERCEPTION_DEBUG && AR_DETECTION
      imshow("depth", src);
      waitKey(1);  
    #endif

    #endif

/* --- Publish LCMs --- */
  lcm_.publish("/target_position_list", &arTagsMessage);
  #if PERCEPTION_DEBUG
  cerr << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!Tag 1 Location Sent: " << arTags[0].x << " " << arTags[0].y << " " << arTags[0].z <<"\n";
  cerr << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!Tag 1 ID Sent: " << arTags[0].target_id << "\n";
    
   //print out second tag if found
   //if(arTags[1].target_id != -1) {
        cerr << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!Tag 2 Location Sent: " << arTags[1].x << " " << arTags[1].y << " " << arTags[1].z  <<"\n";
        cerr << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!Tag 2 ID Sent: " << arTags[1].target_id << "\n";
   // } 
   #endif

    #if !ZED_SDK_PRESENT
    std::this_thread::sleep_for(1s); // Iteration speed control not needed when using camera 
    #endif
    cerr<<"LCM sent"<<endl;
  }
  cerr<<"process completed"<<endl;
/* --- Wrap Things Up --- */
  #if AR_RECORD
    cam.record_ar_finish();
  #endif

 return 0;

}
