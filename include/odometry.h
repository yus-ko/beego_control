//多重インクルード防止
#ifndef INCLUDE_ODOMETRY_CLASS
#define INCLUDE_ODOMETRY_CLASS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <beego_control/odometryConfig.h>
#include <beego_control/rpc.h>
#include <beego_control/vel.h>

//クラスの定義
class odometryClass{
    private:
        //センサーデータ
        ros::NodeHandle nhSub;
        ros::Subscriber subOdom,subAcc,subVel;
        tf::TransformBroadcaster odom_broadcaster;
        geometry_msgs::TransformStamped odom_trans;
        beego_control::rpc rpc;
        beego_control::vel vel;
        std_msgs::Int32 acc;
        //送信データ
		ros::NodeHandle nhPub;
        ros::Publisher pub;
        //--rqt_reconfigure
        dynamic_reconfigure::Server<beego_control::odometryConfig> server;
        dynamic_reconfigure::Server<beego_control::odometryConfig>::CallbackType f;
        //--syncro subscribe
        typedef message_filters::sync_policies::ApproximateTime<beego_control::rpc, beego_control::vel, std_msgs::Int32> MySyncPolicy;
        message_filters::Synchronizer<MySyncPolicy> sync;
    public:
        //in constracter.cpp
        //コンストラクタ：クラス定義に呼び出されるメソッド
        odometryClass();
        //デストラクタ：クラスが消滅するときに呼びだされるメソッド
        ~odometryClass();
        //
        //メソッド：関数のようなもの:後でlaunchファイルからの読み込みメソッドを追加
        //in property.cpp
        //セット：内部パラメータの書き込み
        void setParam(int& temp);//(未使用)
        //セット：内部パラメータの書き込み
        void setLaunchParam();//launchファイルからの探索窓パラメータ書き込み
        //
        //in methods.cpp
        void manage();
        //その他メソッド
        //--センサーデータ受信
        void encorder_callback(const beego_control::rpc::ConstPtr& rpcMsg, const beego_control::vel::ConstPtr& velMsg, const std_msgs::Int32::ConstPtr& accMsg);
        //データ送信
        void sendTransform();
};
#endif
