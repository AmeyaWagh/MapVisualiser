/*
 * Copyright (c) 2010, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// %Tag(FULLTEXT)%
#include <ros/ros.h>
// #include <visualization_msgs/Marker.h>
#include <sensor_msgs/Range.h>
#include <stdio.h>

#include <cmath>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "points_and_lines");
  ros::NodeHandle n;
  // ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  ros::Publisher sensor_pub1 = n.advertise<sensor_msgs::Range>("sensor_range1", 10);
  ros::Publisher sensor_pub2 = n.advertise<sensor_msgs::Range>("sensor_range2", 10);
  ros::Publisher sensor_pub3 = n.advertise<sensor_msgs::Range>("sensor_range3", 10);
  ros::Publisher sensor_pub4 = n.advertise<sensor_msgs::Range>("sensor_range4", 10);
  ros::Publisher sensor_pub5 = n.advertise<sensor_msgs::Range>("sensor_range5", 10);
  ros::Rate r(0.5);
  
  sensor_msgs::Range range_msg1;
  sensor_msgs::Range range_msg2;
  sensor_msgs::Range range_msg3;
  sensor_msgs::Range range_msg4;
  sensor_msgs::Range range_msg5;
  
  

  float f = 0.0;
  while (ros::ok())
  {
// %Tag(MARKER_INIT)%
    // visualization_msgs::Marker points, line_strip, line_list;
    // points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "/my_frame";
    // points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
    // points.ns = line_strip.ns = line_list.ns = "points_and_lines";
    // points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
    // points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;
// %EndTag(MARKER_INIT)%

    range_msg1.radiation_type = sensor_msgs::Range::INFRARED;
    range_msg1.header.frame_id =  "/my_frame1";
    range_msg1.field_of_view = 0.01;
    range_msg1.min_range = 0.03;  // For GP2D120XJ00F only. Adjust for other IR rangers
    range_msg1.max_range = 0.4;   // For GP2D120XJ00F only. Adjust for other IR rangers
    range_msg1.header.stamp = ros::Time::now();

    range_msg2.radiation_type = sensor_msgs::Range::INFRARED;
    range_msg2.header.frame_id =  "/my_frame2";
    range_msg2.field_of_view = 0.01;
    range_msg2.min_range = 0.03;  // For GP2D120XJ00F only. Adjust for other IR rangers
    range_msg2.max_range = 0.4;   // For GP2D120XJ00F only. Adjust for other IR rangers
    range_msg2.header.stamp = ros::Time::now();

    range_msg3.radiation_type = sensor_msgs::Range::INFRARED;
    range_msg3.header.frame_id =  "/my_frame3";
    range_msg3.field_of_view = 0.01;
    range_msg3.min_range = 0.03;  // For GP2D120XJ00F only. Adjust for other IR rangers
    range_msg3.max_range = 0.4;   // For GP2D120XJ00F only. Adjust for other IR rangers
    range_msg3.header.stamp = ros::Time::now();

    range_msg4.radiation_type = sensor_msgs::Range::INFRARED;
    range_msg4.header.frame_id =  "/my_fram4";
    range_msg4.field_of_view = 0.01;
    range_msg4.min_range = 0.03;  // For GP2D120XJ00F only. Adjust for other IR rangers
    range_msg4.max_range = 0.4;   // For GP2D120XJ00F only. Adjust for other IR rangers
    range_msg4.header.stamp = ros::Time::now();

    range_msg5.radiation_type = sensor_msgs::Range::INFRARED;
    range_msg5.header.frame_id =  "/my_fram5";
    range_msg5.field_of_view = 0.01;
    range_msg5.min_range = 0.03;  // For GP2D120XJ00F only. Adjust for other IR rangers
    range_msg5.max_range = 0.4;   // For GP2D120XJ00F only. Adjust for other IR rangers
    range_msg5.header.stamp = ros::Time::now();
// %Tag(ID)%
//     points.id = 0;
//     line_strip.id = 1;
//     line_list.id = 2;
// // %EndTag(ID)%

// // %Tag(TYPE)%
//     points.type = visualization_msgs::Marker::POINTS;
//     line_strip.type = visualization_msgs::Marker::LINE_STRIP;
//     line_list.type = visualization_msgs::Marker::LINE_LIST;
// // %EndTag(TYPE)%

// // %Tag(SCALE)%
//     // POINTS markers use x and y scale for width/height respectively
//     points.scale.x = 0.2;
//     points.scale.y = 0.2;

//     // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
//     line_strip.scale.x = 0.1;
//     line_list.scale.x = 0.1;
// // %EndTag(SCALE)%

// // %Tag(COLOR)%
//     // Points are green
//     points.color.g = 1.0f;
//     points.color.a = 1.0;

//     // Line strip is blue
//     line_strip.color.b = 1.0;
//     line_strip.color.a = 1.0;

//     // Line list is red
//     line_list.color.r = 1.0;
//     line_list.color.a = 1.0;
// // %EndTag(COLOR)%

// %Tag(HELIX)%
    // Create the vertices for the points and lines
    float k=3.0;
    for (uint32_t i = 0; i < 100; ++i)
    {
      // float y = 5 * sin(f + i / 100.0f * 2 * M_PI);
      // float z = 5 * cos(f + i / 100.0f * 2 * M_PI);

      // geometry_msgs::Point p;
      // p.x = (int32_t)i - 50;
      // p.y = y;
      // p.z = z;

      // points.points.push_back(p);
      // line_strip.points.push_back(p);

      // // The line list needs two points for each line
      // line_list.points.push_back(p);
      // p.z += 1.0;
      // line_list.points.push_back(p);
      if(k>40.0)
      {
        k=3.0;
      }
      range_msg1.range = k/100.0;
      range_msg2.range = k/100.0;
      range_msg3.range = k/100.0;
      range_msg4.range = k/100.0;
      range_msg5.range = k/100.0;
      k = k+1.0;
      // printf("value %f\n",k/100.0);
      sensor_pub1.publish(range_msg1);
      sensor_pub2.publish(range_msg2);
      sensor_pub3.publish(range_msg3);
      sensor_pub4.publish(range_msg4);
      sensor_pub5.publish(range_msg5);
      r.sleep();
    }
// %EndTag(HELIX)%

    // marker_pub.publish(points);
    // marker_pub.publish(line_strip);
    // marker_pub.publish(line_list);


    f += 0.04;
  }
}
// %EndTag(FULLTEXT)%
