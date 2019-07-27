//多重インクルード防止
#ifndef INCLUDE_ODOMETRY_CLASS
#define INCLUDE_ODOMETRY_CLASS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <beego_control/odometryConfig.h>
#include <beego_control/rpc.h>
#include <beego_control/vel.h>
#include <beego_control/beego_encoder.h>
#include<std_msgs/Int32.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/Image.h>

//クラスの定義
class odometryClass{
    private:
        //センサーデータ
		ros::Subscriber sub;
        ros::NodeHandle nhSub;
        tf::TransformBroadcaster odom_broadcaster;
        geometry_msgs::TransformStamped odom_trans;
        beego_control::beego_encoder encoder;
        //送信データ
		ros::NodeHandle nhPub;
        ros::Publisher pub;
        //--rqt_reconfigure
        dynamic_reconfigure::Server<beego_control::odometryConfig> server;
        dynamic_reconfigure::Server<beego_control::odometryConfig>::CallbackType f;
        //--odometry param
        nav_msgs::Odometry initOdom;
        nav_msgs::Odometry difOdom;
        double d;
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
        void culcOdometry();
        void setOdometry();
        void manage();
        //その他メソッド
        //--センサーデータ受信
        //void encoder_callback(const beego_control::rpc::ConstPtr& rpcMsg, const beego_control::vel::ConstPtr& velMsg, const std_msgs::Int32::ConstPtr& accMsg);
        void encoder_callback(const beego_control::beego_encoder::ConstPtr& msg);
        void configCallback(beego_control::odometryConfig &config, uint32_t level);

        //データ送信
        void sendTransform();
};
#endif
