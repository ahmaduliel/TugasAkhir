 #include <ros/ros.h>
 #include <visualization_msgs/Marker.h>
 
 int main( int argc, char** argv )
 {
    ros::init(argc, argv, "points_and_lines");
    ros::NodeHandle n;
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  
    ros::Rate r(30);
  
    float f = 0.0;
    while (ros::ok())
    {
 
     visualization_msgs::Marker points, text, arah, line;
     points.header.frame_id = text.header.frame_id = arah.header.frame_id = line.header.frame_id = "/map";
     //points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
     points.ns = text.ns = arah.ns = line.ns = "points_and_lines";
     points.action = text.action = arah.action = line.action = visualization_msgs::Marker::ADD;
     points.pose.orientation.w = text.pose.orientation.w = arah.pose.orientation.w = line.pose.orientation.w = 1.0;
 
     points.id = 0;
     text.id = 1;
     arah.id = 2;
     line.id = 3;

     points.type = visualization_msgs::Marker::POINTS;
     text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
     arah.type = visualization_msgs::Marker::ARROW;
     line.type = visualization_msgs::Marker::LINE_STRIP;

     arah.pose.position.x = -0.15;
     arah.pose.position.y = 0.2;
     arah.pose.position.z = 0.0;
     arah.pose.orientation.x = 0.0;
     arah.pose.orientation.y = 0.0;
     arah.pose.orientation.z = 0.0;
     arah.pose.orientation.w = 1.0;

     arah.scale.x = 0.3;
     arah.scale.y = 0.03;
     arah.scale.z = 0.08;

     arah.color.r = 0.0f;
     arah.color.g = 1.0f;
     arah.color.b = 0.0f;
     arah.color.a = 1.0;

      // POINTS markers use x and y scale for width/height respectively
     points.scale.x = 0.05;
     points.scale.y = 0.05;
     
     line.scale.x = 0.03;

     text.pose.position.x = 0.0;
     text.pose.position.y = 1.0;
     text.pose.position.z = 0.0;
     text.pose.orientation.x = 0.0;
     text.pose.orientation.y = 0.0;
     text.pose.orientation.z = 0.0;
     text.pose.orientation.w = 1.0;

     text.text = "blablabla";

     text.scale.x = 0.3;
     text.scale.y = 0.3;
     text.scale.z = 0.1;

     text.color.r = 0.0f;
     text.color.g = 1.0f;
     text.color.b = 0.0f;
     text.color.a = 1.0;
     
     // Points are green
     points.color.g = 1.0f;
     points.color.a = 1.0;
 
     line.color.b = 1.0f;
     line.color.a = 1.0;
 
      for (uint32_t i = 0; i < 3; ++i)
       {
         geometry_msgs::Point p;
         p.x = i + 1;
         p.y = 0;
         p.z = 0;
 
         points.points.push_back(p);
         line.points.push_back(p);
       }
     marker_pub.publish(text);
     marker_pub.publish(points);
     marker_pub.publish(arah);
     marker_pub.publish(line);

     r.sleep();
 
    }
}