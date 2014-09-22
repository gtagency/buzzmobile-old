#include <ros/ros.h>
#include <sensor_msgs/sensor_msgs.h>
#include "msgs/LaneTrain.h"
#include "msgs/Lane.h"
#include "msgs/LaneArray.h"
#include "image.h"
#include "classifier.h"
#include "score.h"
#include "profiler.h"

ros::Publisher lanes_pub;

Classifier *c;

void trainingCallback(const LaneTrain::ConstPtr& training) {
  //convert to Instance objects
  // then: 
  // c->addInstances(instances);
  // maybe cull/retire old instances
}

void imageCallback(const sensor_msgs::Image::ConstPtr& image) {
  
 //this classifies back into an image...instead we want to classify into a set of points
 // and then publish the points as a lane or something that can be interpreted as a lane
 /* 
    marked = Mat(src.size(), src.type());
    Mat lab;
    if (getCvType() >= 0) {
        cvtColor(src, lab, getCvType());
    } else {
        lab = src;
    }
    uchar features[2];
    for(int row = 0; row < lab.rows; ++row) {
        //cout << "Row: " << row << endl;
        Point3_<uchar> *p = lab.ptr<Point3_<uchar> > (row);
        Point3_<uchar> *sp = marked.ptr<Point3_<uchar> >(row);
        //assumes CV_8UC3 LAB color image, with 3 values per pixel
        for(int col = 0; col < lab.cols; ++col, ++p, ++sp) {
            //throw away the L
//            cout << "Col: " << col << endl;

            getFeatures(p, features);
            const Instance inst = makeInstance(features, abs(col - inLaneStart), -1);
            int label = c.classify(inst);    
            if (row == 2) {
                //cout << "Col " << col << ": " << (int)p->x << ", " << (int)p->y << ", " << (int)p->z << ", " << label << endl;
            }
            sp->x = 0;
            //cout << countPositives << endl;
            if (label) {
                sp->y = 0xFF;
                sp->z = 0;
            } else {
                sp->y = 0;
                sp->z = 0;
            }
        }
    }   
*/
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "lane_classifier");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe<LaneTrain>("road_class_train", 1000, trainingCallback);
  ros::Subscriber sub = n.subscribe<sensor_msgs::Image>("image_projected", 1000, imageCallback);

  int k = 21;
  int data[3] = {0};
  Evaluation eval;
  eval._data  = data;
  eval._score = scoreHueAndSat;
  c = new Classifier(new Profiler(), k, eval);

  lanes_pub = n.advertise<LaneArray>("lanes", 100);
  ros::spin();
  delete c;
  return 0;
}
