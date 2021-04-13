//
// Created by user on 21/03/2021.
//

#ifndef SRC_MARKERS_INIT_H
#define SRC_MARKERS_INIT_H

#endif //SRC_MARKERS_INIT_H
#ifndef SOURCE_FRAME
#define SOURCE_FRAME "base_link"
#endif
#ifndef TARGET_FRAME
#define TARGET_FRAME "link_depthtf"
#endif
void initCloudMarker(visualization_msgs::Marker &point_cloud_marker)
{
    point_cloud_marker.header.frame_id = SOURCE_FRAME;
    point_cloud_marker.header.stamp = ros::Time();
    point_cloud_marker.ns = "my_namespace";
    point_cloud_marker.id = 0;
    point_cloud_marker.type = visualization_msgs::Marker::CUBE_LIST;
    point_cloud_marker.action = visualization_msgs::Marker::ADD;



    point_cloud_marker.pose.position.x = 0;
    point_cloud_marker.pose.position.y = 0;
    point_cloud_marker.pose.position.z = 0;
    point_cloud_marker.pose.orientation.x = 0.0;
    point_cloud_marker.pose.orientation.y = 0.0;
    point_cloud_marker.pose.orientation.z = 0.0;
    point_cloud_marker.pose.orientation.w = 1.0;
    point_cloud_marker.scale.x = 0.01;
    point_cloud_marker.scale.y = 0.01;
    point_cloud_marker.scale.z = 0.01;
    point_cloud_marker.color.a = 1.0; // Don't forget to set the alpha!
    point_cloud_marker.color.r = 0.0;
    point_cloud_marker.color.g = 1.0;
    point_cloud_marker.color.b = 0.0;
}

void initLineMarker(visualization_msgs::Marker &position_marker)
{
    position_marker.header.frame_id = SOURCE_FRAME;
    position_marker.header.stamp = ros::Time();
    position_marker.ns = "my_namespace";
    position_marker.id = 0;
    position_marker.type = visualization_msgs::Marker::LINE_LIST;
    position_marker.action = visualization_msgs::Marker::ADD;

    position_marker.pose.position.x = 0;
    position_marker.pose.position.y = 0;
    position_marker.pose.position.z = 0;
    position_marker.pose.orientation.x = 0.0;
    position_marker.pose.orientation.y = 0.0;
    position_marker.pose.orientation.z = 0.0;
    position_marker.pose.orientation.w = 1.0;
    position_marker.scale.x = 0.01;
    position_marker.scale.y = 0.01;
    position_marker.scale.z = 0.01;
    position_marker.color.a = 1.0; // Don't forget to set the alpha!
    position_marker.color.r = 0.0;
    position_marker.color.g = 0.0;
    position_marker.color.b = 1.0;
}