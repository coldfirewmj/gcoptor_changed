#include "misc/visualizer.hpp"
#include "gcopter/trajectory.hpp"
#include "gcopter/gcopter.hpp"
#include "gcopter/firi.hpp"
#include "gcopter/flatness.hpp"
#include "gcopter/voxel_map.hpp"
#include "gcopter/sfc_gen.hpp"
#include "dyn_a_star.h"
#include <visualization_msgs/Marker.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <chrono>
#include <random>

struct Config
{
    double sfc_Range;

    std::string mapTopic;
    std::string targetTopic;
    double dilateRadius;
    double voxelWidth;
    std::vector<double> mapBound;
    double timeoutRRT;
    double maxVelMag;
    double maxBdrMag;
    double maxTiltAngle;
    double minThrust;
    double maxThrust;
    double vehicleMass;
    double gravAcc;
    double horizDrag;
    double vertDrag;
    double parasDrag;
    double speedEps;
    double weightT;
    std::vector<double> chiVec;
    double smoothingEps;
    int integralIntervs;
    double relCostTol;

    Config(const ros::NodeHandle &nh_priv)
    {
        nh_priv.getParam("SFCrange", sfc_Range);
        nh_priv.getParam("MapTopic", mapTopic);
        nh_priv.getParam("TargetTopic", targetTopic);
        nh_priv.getParam("DilateRadius", dilateRadius);
        nh_priv.getParam("VoxelWidth", voxelWidth);
        nh_priv.getParam("MapBound", mapBound);
        nh_priv.getParam("TimeoutRRT", timeoutRRT);
        nh_priv.getParam("MaxVelMag", maxVelMag);
        nh_priv.getParam("MaxBdrMag", maxBdrMag);
        nh_priv.getParam("MaxTiltAngle", maxTiltAngle);
        nh_priv.getParam("MinThrust", minThrust);
        nh_priv.getParam("MaxThrust", maxThrust);
        nh_priv.getParam("VehicleMass", vehicleMass);
        nh_priv.getParam("GravAcc", gravAcc);
        nh_priv.getParam("HorizDrag", horizDrag);
        nh_priv.getParam("VertDrag", vertDrag);
        nh_priv.getParam("ParasDrag", parasDrag);
        nh_priv.getParam("SpeedEps", speedEps);
        nh_priv.getParam("WeightT", weightT);
        nh_priv.getParam("ChiVec", chiVec);
        nh_priv.getParam("SmoothingEps", smoothingEps);
        nh_priv.getParam("IntegralIntervs", integralIntervs);
        nh_priv.getParam("RelCostTol", relCostTol);
    }
};

struct UAVState{
    Eigen::Vector3d pos;
    Eigen::Vector3d vel;
    Eigen::Vector3d acc;
    double yaw;
};

class GlobalPlanner
{
private:
    Config config;
    UAVState state;

    ros::NodeHandle nh;
    ros::Subscriber mapSub;
    ros::Subscriber targetSub,airsim_targetsub,airsim_odomsub;
    ros::Publisher slice_pub_;

    bool mapInitialized,airsimgoal_get,odomget;
    voxel_map::VoxelMap voxelMap;

    Visualizer visualizer;
    std::vector<Eigen::Vector3d> startGoal;
    Eigen::Vector3d odom;

    Trajectory<5> traj;
    double trajStamp;
public:
    double veladd;
    double jerkadd;
    std::vector<double> alljerk;
    int timeadd;
    std::vector<Eigen::Vector3d> record_route;
    std::vector<Eigen::MatrixX4d> record_hPolys;
    AStar a_star_;
    GlobalPlanner(const Config &conf,
                  ros::NodeHandle &nh_)
        : config(conf),
          nh(nh_),
          mapInitialized(false),
          airsimgoal_get(false),
          odomget(false),
          visualizer(nh)
    {
        //std::cout<<"voxelWidth is :"<<config.voxelWidth<<std::endl;
        std::cout<<"mapBound is :"<<config.mapBound[0]<<" "<<config.mapBound[1]<<"  "<<config.mapBound[2]<<"  "<<config.mapBound[3]<<"  "<<config.mapBound[4]<<"  "<<config.mapBound[5]<<std::endl;
        std::cout<<"chiVec is :"<<config.chiVec[0]<<" "<<config.chiVec[1]<<"  "<<config.chiVec[2]<<"  "<<config.chiVec[3]<<"  "<<config.chiVec[4]<<std::endl;
        const Eigen::Vector3i xyz((config.mapBound[1] - config.mapBound[0]) / config.voxelWidth,
                                  (config.mapBound[3] - config.mapBound[2]) / config.voxelWidth,
                                  (config.mapBound[5] - config.mapBound[4]) / config.voxelWidth);

        const Eigen::Vector3d offset(config.mapBound[0], config.mapBound[2], config.mapBound[4]);

        voxelMap = voxel_map::VoxelMap(xyz, offset, config.voxelWidth);

        mapSub = nh.subscribe(config.mapTopic, 1, &GlobalPlanner::mapCallBack, this,
                              ros::TransportHints().tcpNoDelay());

        targetSub = nh.subscribe(config.targetTopic, 1, &GlobalPlanner::targetCallBack, this,
                                 ros::TransportHints().tcpNoDelay());
        airsim_targetsub=nh.subscribe("/airsim/goal", 1, &GlobalPlanner::airsimtargetCallBack, this);
        airsim_odomsub=nh.subscribe("/airsim/odom", 1, &GlobalPlanner::airsimodomCallBack, this);
        slice_pub_ = nh.advertise<visualization_msgs::Marker>("ESDFMap/slice", 1, true);
    }

    inline void mapCallBack(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        if (!mapInitialized)
        {
            size_t cur = 0;
            const size_t total = msg->data.size() / msg->point_step;
            float *fdata = (float *)(&msg->data[0]);
            for (size_t i = 0; i < total; i++)
            {
                cur = msg->point_step / sizeof(float) * i;

                if (std::isnan(fdata[cur + 0]) || std::isinf(fdata[cur + 0]) ||
                    std::isnan(fdata[cur + 1]) || std::isinf(fdata[cur + 1]) ||
                    std::isnan(fdata[cur + 2]) || std::isinf(fdata[cur + 2]))
                {
                    continue;
                }
                voxelMap.setOccupied(Eigen::Vector3d(fdata[cur + 0],
                                                     fdata[cur + 1],
                                                     fdata[cur + 2]));
            }

            voxelMap.dilate(std::ceil(config.dilateRadius / voxelMap.getScale()));
            ros::Time start = ros::Time::now();
            //voxelMap.update_esdf();
            ros::Time end = ros::Time::now();
            std::cout<<"the ESDF calculate time is: "<<(end-start).toSec()*1000<<std::endl;
            a_star_.initGridMap(voxelMap);

            mapInitialized = true;
        }
    }

    inline std_msgs::ColorRGBA RainbowColorMap(double h) {
        std_msgs::ColorRGBA color;
        color.a = 1;
        // blend over HSV-values (more colors)

        double s = 1.0;
        double v = 1.0;

        h -= floor(h);
        h *= 6;
        int i;
        double m, n, f;

        i = floor(h);
        f = h - i;
        if (!(i & 1))
            f = 1 - f;  // if i is even
        m = v * (1 - s);
        n = v * (1 - s * f);

        switch (i) {
            case 6:
            case 0:color.r = v;
            color.g = n;
            color.b = m;
            break;
            case 1:color.r = n;
            color.g = v;
            color.b = m;
            break;
            case 2:color.r = m;
            color.g = v;
            color.b = n;
            break;
            case 3:color.r = m;
            color.g = n;
            color.b = v;
            break;
            case 4:color.r = n;
            color.g = m;
            color.b = v;
            break;
            case 5:color.r = v;
            color.g = m;
            color.b = n;
            break;
            default:color.r = 1;
            color.g = 0.5;
            color.b = 0.5;
            break;
        }

        return color;
    }

    inline void plan()
    {
        if (startGoal.size() == 2)
        {
            std::vector<Eigen::Vector3d> route,theta,true_route;
            visualization_msgs::Marker slice_marker;
            slice_marker.header.frame_id = "odom";
            slice_marker.id = 101;
            slice_marker.type = visualization_msgs::Marker::POINTS;
            slice_marker.action = visualization_msgs::Marker::MODIFY;
            slice_marker.scale.x = config.voxelWidth;
            slice_marker.scale.y = config.voxelWidth;
            slice_marker.scale.z = config.voxelWidth;
            slice_marker.pose.orientation.w = 1;
            slice_marker.pose.orientation.x = 0;
            slice_marker.pose.orientation.y = 0;
            slice_marker.pose.orientation.z = 0;
            slice_marker.points.clear();
            slice_marker.colors.clear();
            std_msgs::ColorRGBA c;
            Eigen::Vector3i mapsize=voxelMap.getSize();
            for (int x = 0; x <= mapsize(0); ++x){
                for (int y = 0; y <= mapsize(1); ++y) {
                    int z = 13;
                    Eigen::Vector3i vox = Eigen::Vector3i(x, y, z);
                    Eigen::Vector3d vox_d=Eigen::Vector3d(x, y, z);
                    //distance_buffer_大多数为0
                    if (voxelMap.get_esdf(vox) == 0)
                        continue;
                    Eigen::Vector3d pos;
                    pos=config.voxelWidth*vox_d+voxelMap.getOrigin();

                    geometry_msgs::Point p;
                    p.x = pos(0);
                    p.y = pos(1);
                    p.z = pos(2);
                    c = RainbowColorMap(
                        voxelMap.get_esdf(vox) <= config.voxelWidth*10 ? voxelMap.get_esdf(vox) / (config.voxelWidth*10) : 1);
                    slice_marker.points.push_back(p);
                    slice_marker.colors.push_back(c);
                }
            }
            slice_pub_.publish(slice_marker);
            ros::Time start = ros::Time::now();

            ASTAR_RET ret = a_star_.AstarSearch(config.voxelWidth, startGoal[0],startGoal[1]);
            ros::Time end = ros::Time::now();

            std::cout<<"the Astar calculate time is: "<<(end-start).toSec()*1000<<std::endl;
            start=end;
            /*sfc_gen::setup<voxel_map::VoxelMap>(voxelMap.getOrigin(),
                                                voxelMap.getCorner(),
                                                &voxelMap);

            sfc_gen::planPath<voxel_map::VoxelMap>(startGoal[0],
                                                   startGoal[1],
                                                   voxelMap.getOrigin(),
                                                   voxelMap.getCorner(),
                                                   &voxelMap, 1,
                                                   route);*/
            if (ret == ASTAR_RET::SUCCESS)
            {
                //std::cout<<"this is astar path"<<std::endl;
                route=a_star_.getPath();
                //std::cout<<"route[0] is: "<<route[0].transpose()<<"route end is: "<<route[route.size()-1].transpose()<<std::endl;
            }
            else
            {
                std::cout<<"astar path fail"<<std::endl;
            }
            /*std::cout<<"the astar path value is:"<<std::endl;
            for (int i=0;i<route.size();i++)
            {
                Eigen::Vector3i vox;
                vox=((route[i] - voxelMap.getOrigin()) / config.voxelWidth).cast<int>();
                double value=voxelMap.get_esdf(vox);
                std::cout<<value<<std::endl;
            }*/
            route.insert(route.begin(),startGoal[0]);
            route.emplace_back(startGoal[1]);
            theta=a_star_.Thetastar(route);
            std::cout<<"theta path size is: "<<theta.size()<<std::endl;

            std::vector<Eigen::MatrixX4d> hPolys;
            std::vector<Eigen::Vector3d> pc;
            true_route.reserve(route.size());
            double enlarge=10.0;
            std::cout<<"route size is: "<<route.size()<<std::endl;
            for (int i=0;i<route.size();i++)
            {
                true_route.push_back(route[i]);
            }
            //pc=voxelMap.new_surf(2.5);
            voxelMap.getSurf(pc);
            sfc_gen::convexCover(true_route,
                                 pc,
                                 voxelMap.getOrigin(),
                                 voxelMap.getCorner(),
                                 7.0*enlarge,
                                 config.sfc_Range*enlarge,
                                 hPolys);           
            sfc_gen::shortCut(hPolys);
            end = ros::Time::now();
            std::cout<<"the SFC calculate time is: "<<(end-start).toSec()*1000<<std::endl;
            start=end;
            std::cout<<"the hPolys size is: "<<hPolys.size()<<std::endl;
            if (true_route.size() > 1)
            {
                visualizer.visualizePolytope(hPolys);
                record_hPolys=hPolys;
                Eigen::Matrix3d iniState;
                Eigen::Matrix3d finState;
                iniState << true_route.front(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
                finState << true_route.back(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();

                gcopter::GCOPTER_PolytopeSFC gcopter;

                // magnitudeBounds = [v_max, omg_max, theta_max, thrust_min, thrust_max]^T
                // penaltyWeights = [pos_weight, vel_weight, omg_weight, theta_weight, thrust_weight]^T
                // physicalParams = [vehicle_mass, gravitational_acceleration, horitonral_drag_coeff,
                //                   vertical_drag_coeff, parasitic_drag_coeff, speed_smooth_factor]^T
                // initialize some constraint parameters
                Eigen::VectorXd magnitudeBounds(5);
                Eigen::VectorXd penaltyWeights(5);
                Eigen::VectorXd physicalParams(6);
                magnitudeBounds(0) = config.maxVelMag;
                magnitudeBounds(1) = config.maxBdrMag;
                magnitudeBounds(2) = config.maxTiltAngle;
                magnitudeBounds(3) = config.minThrust;
                magnitudeBounds(4) = config.maxThrust;
                penaltyWeights(0) = (config.chiVec)[0];
                penaltyWeights(1) = (config.chiVec)[1];
                penaltyWeights(2) = (config.chiVec)[2];
                penaltyWeights(3) = (config.chiVec)[3];
                penaltyWeights(4) = (config.chiVec)[4];
                physicalParams(0) = config.vehicleMass;
                physicalParams(1) = config.gravAcc;
                physicalParams(2) = config.horizDrag;
                physicalParams(3) = config.vertDrag;
                physicalParams(4) = config.parasDrag;
                physicalParams(5) = config.speedEps;
                const int quadratureRes = config.integralIntervs;

                traj.clear();

                if (!gcopter.setup(config.weightT,
                                   iniState, finState,
                                   hPolys, INFINITY,
                                   config.smoothingEps,
                                   quadratureRes,
                                   magnitudeBounds,
                                   penaltyWeights,
                                   physicalParams))
                {
                    return;
                }

                if (std::isinf(gcopter.optimize(traj, config.relCostTol)))
                {
                    return;
                }
                ros::Time end = ros::Time::now();
                std::cout<<"the TRAJ calculate time is: "<<(end-start).toSec()*1000<<std::endl;
                if (traj.getPieceNum() > 0)
                {
                    std::cout<<"traj piecenum is:"<<traj.getPieceNum()<<std::endl;
                    trajStamp = ros::Time::now().toSec();
                    veladd=0;
                    record_route=theta;
                    visualizer.visualize(traj, theta);
                }
            }
            timeadd=0;
        }
    }

    inline void targetCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        if (mapInitialized)
        {
            if (startGoal.size() >= 2)
            {
                startGoal.clear();
            }
            const double zGoal = config.mapBound[4] + config.dilateRadius +
                                 fabs(msg->pose.orientation.z) *
                                     (config.mapBound[5] - config.mapBound[4] - 2 * config.dilateRadius); 
            const Eigen::Vector3d goal(msg->pose.position.x, msg->pose.position.y, zGoal);
            if (voxelMap.query(goal) == 0)
            {
                visualizer.visualizeStartGoal(goal, 0.5, startGoal.size());
                startGoal.emplace_back(goal);
            }
            else
            {
                ROS_WARN("Infeasible Position Selected !!!\n");
            }

            plan();
        }
        return;
    }

    inline void airsimtargetCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        if(!airsimgoal_get&&odomget)
        {
            if (mapInitialized)
            {
                airsimgoal_get=true;
                const Eigen::Vector3d goal(msg->pose.position.x, msg->pose.position.y, odom(2));
                if (voxelMap.query(goal) == 0)
                {
                    visualizer.visualizeStartGoal(goal, 0.5, startGoal.size());
                    startGoal.emplace_back(odom);
                    startGoal.emplace_back(goal);
                }
                else
                {
                    ROS_WARN("Infeasible Position Selected !!!\n");
                }
                plan();
            }
        }
        return;
    }

    inline void airsimodomCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        if (mapInitialized)
        {
            odomget=true;
            Eigen::Vector3d start(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
            odom=start;
        }
    }

    inline void process()
    {
        Eigen::VectorXd physicalParams(6);
        physicalParams(0) = config.vehicleMass;
        physicalParams(1) = config.gravAcc;
        physicalParams(2) = config.horizDrag;
        physicalParams(3) = config.vertDrag;
        physicalParams(4) = config.parasDrag;
        physicalParams(5) = config.speedEps;

        flatness::FlatnessMap flatmap;
        flatmap.reset(physicalParams(0), physicalParams(1), physicalParams(2),
                      physicalParams(3), physicalParams(4), physicalParams(5));

        if (traj.getPieceNum() > 0)
        {
            const double delta = ros::Time::now().toSec() - trajStamp;
            if (delta > 0.0 && delta < traj.getTotalDuration())
            {
                double thr;
                Eigen::Vector4d quat;
                Eigen::Vector3d omg;

                state.pos=traj.getPos(delta);
                state.vel=traj.getVel(delta);
                state.acc=traj.getAcc(delta);
                //state.yaw=traj.;

                flatmap.forward(traj.getVel(delta),
                                traj.getAcc(delta),
                                traj.getJer(delta),
                                0.0, 0.0,
                                thr, quat, omg);
                double speed = traj.getVel(delta).norm();
                double jerk  = traj.getJer(delta).norm();
                veladd+=speed;
                jerkadd+=jerk*jerk;
                alljerk.push_back(jerk);
                timeadd+=1;
                double bodyratemag = omg.norm();
                double tiltangle = acos(1.0 - 2.0 * (quat(1) * quat(1) + quat(2) * quat(2)));
                std_msgs::Float64 speedMsg, thrMsg, tiltMsg, bdrMsg;
                speedMsg.data = speed;
                thrMsg.data = thr;
                tiltMsg.data = tiltangle;
                bdrMsg.data = bodyratemag;
                visualizer.speedPub.publish(speedMsg);
                visualizer.thrPub.publish(thrMsg);
                visualizer.tiltPub.publish(tiltMsg);
                visualizer.bdrPub.publish(bdrMsg);

                visualizer.visualizeSphere(traj.getPos(delta),
                                           config.dilateRadius);
            }
            else if((delta-traj.getTotalDuration())<0.001)
            {
                std::cout<<"total vel averange is:"<<veladd/timeadd<<std::endl;
                std::cout<<"total vel averange percent is:"<<veladd/timeadd/config.maxVelMag*100<<"%"<<std::endl;
                std::cout<<"total jerk averange is:"<<jerkadd/timeadd<<std::endl;
            }
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "global_planning_node");
    ros::NodeHandle nh_;

    GlobalPlanner global_planner(Config(ros::NodeHandle("~")), nh_);

    ros::Rate lr(50);
    while (ros::ok())
    {
        global_planner.process();
        ros::spinOnce();
        lr.sleep();
    }

    return 0;
}
