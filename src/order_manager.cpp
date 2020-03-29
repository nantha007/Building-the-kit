//
// Created by zeid on 2/27/20.
//

#include "order_manager.h"
#include <osrf_gear/AGVControl.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>



//AriacOrderManager::AriacOrderManager(): arm1_{"arm1"}, arm2_{"arm2"}
AriacOrderManager::AriacOrderManager(): arm1_{"arm1", 0}
{
    order_subscriber_ = order_manager_nh_.subscribe(
            "/ariac/orders", 10,
            &AriacOrderManager::OrderCallback, this);

}

AriacOrderManager::~AriacOrderManager(){}


void AriacOrderManager::OrderCallback(const osrf_gear::Order::ConstPtr& order_msg) {
    ROS_WARN(">>>>> OrderCallback");
    received_orders_.push_back(*order_msg);
}


/**
 * @brief Get the product frame for a product type
 * @param product_type
 * @return
 */
std::string AriacOrderManager::GetProductFrame(std::string product_type, bool conv=false) {
    //--Grab the last one from the list then remove it
    if (conv){
        if (!product_frame_list_conv_.empty() && product_frame_list_conv_.find(product_type)!=product_frame_list_conv_.end()) {
            std::string frame = product_frame_list_conv_[product_type].back();
            ROS_INFO_STREAM("Frame >>>> " << frame);
            product_frame_list_conv_[product_type].pop_back();
            return frame;
        } else {
            return "-1";
        }
    }
    else{
        if (!product_frame_list_.empty() && product_frame_list_.find(product_type)!=product_frame_list_.end()) {
            std::string frame = product_frame_list_[product_type].back();
            ROS_INFO_STREAM("Frame >>>> " << frame);
            product_frame_list_[product_type].pop_back();
            return frame;
        } else {
            return "-1";
        }
    }
}

bool AriacOrderManager::PickAndPlace(const std::pair<std::string,geometry_msgs::Pose> product_type_pose, int agv_id) {
    
    arm1_.SendRobotHome(1);

    std::string product_type = product_type_pose.first;
    ROS_WARN_STREAM("Product type >>>> " << product_type);
    std::string product_frame = this->GetProductFrame(product_type);
    ROS_WARN_STREAM("Product frame >>>> " << product_frame);
    auto part_pose = camera_.GetPartPose("/world",product_frame);


    if(product_type == "pulley_part")
        part_pose.position.z += 0.08;
    //--task the robot to pick up this part
    bool failed_pick = arm1_.PickPart(part_pose);
    ROS_WARN_STREAM("Picking up state " << failed_pick);
    ros::Duration(1).sleep();

    while(!failed_pick){
        auto part_pose = camera_.GetPartPose("/world",product_frame);
        failed_pick = arm1_.PickPart(part_pose);
    }

    arm1_.SendRobotHome(1);
    //--get the pose of the object in the tray from the order
    geometry_msgs::Pose drop_pose = product_type_pose.second;

    geometry_msgs::PoseStamped StampedPose_in,StampedPose_out;

    if(agv_id==1){
        StampedPose_in.header.frame_id = "/kit_tray_1";
        StampedPose_in.pose = drop_pose;
        ROS_INFO_STREAM("StampedPose_int (" << StampedPose_in.pose.position.x <<","<< StampedPose_in.pose.position.y << "," << StampedPose_in.pose.position.z<<")");
        part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
        StampedPose_out.pose.position.z += 0.1;
        // StampedPose_out.pose.position.y -= 0.2;
        ROS_INFO_STREAM("StampedPose_out (" << StampedPose_out.pose.position.x <<","<< StampedPose_out.pose.position.y << "," << StampedPose_out.pose.position.z<<")");

    }
    else{
        StampedPose_in.header.frame_id = "/kit_tray_2";
        StampedPose_in.pose = drop_pose;
        //ROS_INFO_STREAM("StampedPose_in " << StampedPose_in.pose.position.x);
        part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
        StampedPose_out.pose.position.z += 0.1;
        // StampedPose_out.pose.position.y += 0.2;
        //ROS_INFO_STREAM("StampedPose_out " << StampedPose_out.pose.position.x);
    }
    auto result = arm1_.DropPart(StampedPose_out.pose);

    return result;
}


bool AriacOrderManager::PickAndPlaceFromConv(const std::pair<std::string,geometry_msgs::Pose> product_type_pose, int agv_id) {
    int i = 0;
    geometry_msgs::Pose part_pose;
    while(true){
        std::string product_type = product_type_pose.first;
        ROS_WARN_STREAM("Product type >>>> " << product_type);
        int part_num{0};
        while(this->GetProductFrame(product_type, true)=="-1"){
            ros::spinOnce();
            ros::Duration(1).sleep();
            ROS_WARN_STREAM_THROTTLE(20,"Waiting for product to arrive");
            if(camera_.get_break_beam_status(2)){
                // ros::spinOnce();
                // ros::Duration(1.0).sleep();
                product_frame_list_conv_ = camera_.get_product_frame_list_conv();    
                if (this->GetProductFrame(product_type, true)!="-1"){
                    // convBelt_.setConveyorPower(0.0);
                    part_num = camera_.get_break_beam_trig_counter(2);
                    break;
                }
                // convBelt_.setConveyorPower(100.0);
            }
            product_frame_list_conv_ = camera_.get_product_frame_list_conv();    
        }
            
        std::string product_frame = this->GetProductFrame(product_type, true);
        ROS_WARN_STREAM("Product frame >>>> " << product_frame);
        if (i==0){
            part_pose = camera_.GetPartPoseFromConv("/world",product_frame);
        }
        auto temp_pose = part_pose;
        if(product_type == "pulley_part")
                temp_pose.position.z += 0.08;
        temp_pose.position.y = 1.05;
        //--task the robot to pick up this part
        ROS_INFO_STREAM("Moving to part...");
        temp_pose.position.z += 0.03;
        // auto temp_pose_1 = part_pose;
        // temp_pose_1.position.z += 0.015;
        arm1_.GoToTarget(temp_pose);
        while(camera_.get_break_beam_trig_counter(1)!=part_num){
            ros::spinOnce();
        }
        if (product_type == "piston_rod_part"){
            temp_pose.position.z -= 0.015;
            arm1_.GoToTarget(temp_pose);
        }
        bool failed_pick = arm1_.PickPartFromConv(temp_pose);
        ROS_WARN_STREAM("Picking up state " << failed_pick);

        if (failed_pick){
            break;
        }

        arm1_.SendRobotHome(0);
        i++;
    }
    // ros::Duration(0.5).sleep();
    // while((!failed_pick)){

        // failed_pick = arm1_.PickPartFromConv(part_pose);
    // }

    //--get the pose of the object in the tray from the order
    geometry_msgs::Pose drop_pose = product_type_pose.second;

    geometry_msgs::PoseStamped StampedPose_in,StampedPose_out;

    if(agv_id==1){
        StampedPose_in.header.frame_id = "/kit_tray_1";
        StampedPose_in.pose = drop_pose;
        ROS_INFO_STREAM("StampedPose_int (" << StampedPose_in.pose.position.x <<","<< StampedPose_in.pose.position.y << "," << StampedPose_in.pose.position.z<<")");
        part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
        StampedPose_out.pose.position.z += 0.1;
        // StampedPose_out.pose.position.y -= 0.2;
        ROS_INFO_STREAM("StampedPose_out (" << StampedPose_out.pose.position.x <<","<< StampedPose_out.pose.position.y << "," << StampedPose_out.pose.position.z<<")");

    }
    else{
        StampedPose_in.header.frame_id = "/kit_tray_2";
        StampedPose_in.pose = drop_pose;
        //ROS_INFO_STREAM("StampedPose_in " << StampedPose_in.pose.position.x);
        part_tf_listener_.transformPose("/world",StampedPose_in,StampedPose_out);
        StampedPose_out.pose.position.z += 0.1;
        // StampedPose_out.pose.position.y += 0.2;
        //ROS_INFO_STREAM("StampedPose_out " << StampedPose_out.pose.position.x);
    }
    auto result = arm1_.DropPart(StampedPose_out.pose);

    // arm1_.SendRobotHome(0);

    return result;
}


void AriacOrderManager::ExecuteOrder() {
    ROS_WARN(">>>>>> Executing order...");
    //-- used to check if pick and place was successful
    bool pick_n_place_success{false};
    std::list<std::pair<std::string,geometry_msgs::Pose>> failed_parts;

    ros::spinOnce();
    ros::Duration(1.0).sleep();
    product_frame_list_ = camera_.get_product_frame_list();
    for (const auto &order:received_orders_){
        auto order_id = order.order_id;
        auto shipments = order.shipments;
        for (const auto &shipment: shipments){
            auto shipment_type = shipment.shipment_type;
            auto agv = shipment.agv_id.back();//--this returns a char
            //-- if agv is any then we use AGV1, else we convert agv id to int
            //--agv-'0' converts '1' to 1 and '2' to 2
            int agv_id = (shipment.agv_id == "any") ? 1 : agv - '0';

            auto products = shipment.products;
            ROS_INFO_STREAM("Order ID: " << order_id);
            ROS_INFO_STREAM("Shipment Type: " << shipment_type);
            ROS_INFO_STREAM("AGV ID: " << agv_id);
            for (const auto &product: products){
                ros::spinOnce();
                product_type_pose_.first = product.type;
                ROS_INFO_STREAM("Product type: " << product_type_pose_.first);
                product_type_pose_.second = product.pose;
                ROS_INFO_STREAM("Product pose: " << product_type_pose_.second.position.x);
                if (this->GetProductFrame(product_type_pose_.first)=="-1"){
                    pick_n_place_success =  PickAndPlaceFromConv(product_type_pose_, agv_id);
                }
                else{
                    pick_n_place_success = PickAndPlace(product_type_pose_, agv_id);
                }
                // --todo: What do we do if pick and place fails?

            }
            arm1_.SendRobotHome(1);
            SubmitAGV(1, "order_0_shipment_0");
            ROS_INFO_STREAM("Submitting AGV 1");
            int finish=1;
        }
    }
}

void AriacOrderManager::SubmitAGV(int num, std::string shipmentID) {
    std::string s = std::to_string(num);
    ros::ServiceClient start_client =
            order_manager_nh_.serviceClient<osrf_gear::AGVControl>("/ariac/agv"+s);
    if (!start_client.exists()) {
        ROS_INFO("Waiting for the client to be ready...");
        start_client.waitForExistence();
        ROS_INFO("Service started.");
    }

    osrf_gear::AGVControl srv;
    srv.request.shipment_type = shipmentID;
    start_client.call(srv);

    if (!srv.response.success) {
        ROS_ERROR_STREAM("Service failed!");
    } else
        ROS_INFO("Service succeeded.");
}
