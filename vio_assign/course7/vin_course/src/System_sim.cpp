#include "System_sim.h"

#include <pangolin/pangolin.h>


namespace sim {



using namespace std;
using namespace cv;
using namespace pangolin;



System::System(string sConfig_file_, sim::Param* params)
    :bStart_backend(true),params_(params)
{
    string sConfig_file = sConfig_file_ + "euroc_config.yaml";

    cout << std::this_thread::get_id() << std::this_thread::get_id() << "1 System() sConfig_file: " << sConfig_file << endl;
    readParameters(sConfig_file);

    trackerData[0].readIntrinsicParameter(sConfig_file);

    estimator.setParameter();
    ofs_pose.open("./pose_output.txt",fstream::out);
    if(!ofs_pose.is_open())
    {
        cerr << "ofs_pose is not open" << endl;
    }
    // thread thd_RunBackend(&System::process,this);
    // thd_RunBackend.detach();
    cout << std::this_thread::get_id() << std::this_thread::get_id() << "2 System() end" << endl;
}

System::~System()
{
    bStart_backend = false;
    
    pangolin::QuitAll();
    
    m_buf.lock();
    while (!feature_buf.empty())
        feature_buf.pop();
    while (!imu_buf.empty())
        imu_buf.pop();
    m_buf.unlock();

    m_estimator.lock();
    estimator.clearState();
    m_estimator.unlock();

    ofs_pose.close();
}

/**
 * Copied from PinholeCamera.cc
 * 
 * \brief Project a 3D point (\a x,\a y,\a z) to the image plane in (\a u,\a v)
 *
 * \param P 3D point coordinates
 * \param p return value, contains the image point coordinates
 */
bool
System::spaceToPlane(const Eigen::Vector3d& P, Eigen::Vector2d& p,
    double fx, double fy, double cx, double cy ) const
{
    Eigen::Vector2d p_u, p_d;

    double denorm = P(2);
    if (P(2) == 0.0) {
        denorm  = EPSILON * 100;
    }
    // Project points to the normalised plane
    p_u << P(0) / denorm, P(1) / denorm;
    if (std::isnan(p_u(0))) {
        std::cerr << "p: " << P << std::endl;
        return false;
    }
    p_d = p_u;
    //  do not consider distortion here.
    /*if (m_noDistortion)
    {
        p_d = p_u;
    }
    else
    {
        // Apply distortion
        Eigen::Vector2d d_u;
        distortion(p_u, d_u);
        p_d = p_u + d_u;
    }*/

    // Apply generalised projection matrix
    p << fx * p_d(0) + cx,
         fy * p_d(1) + cy;
}



void System::PubImageFts(double dStampSec, vector<SIM_PTS_INFO>& fts) {
    //1. pure fts , no need for optical flow computation.
    //2. sim data should be stable and continuous.
    if (first_image_flag)
    {
        cout << std::this_thread::get_id() << std::this_thread::get_id() << "2 PubImageData first_image_flag" << endl;
        first_image_flag = false;
        first_image_time = dStampSec;
        last_image_time = dStampSec;
        return;
    }

    last_image_time = dStampSec;
    if (round(1.0 * pub_count / (dStampSec - first_image_time)) <= FREQ)
    {
        PUB_THIS_FRAME = true;
        // reset the frequency control
        if (abs(1.0 * pub_count / (dStampSec - first_image_time) - FREQ) < 0.01 * FREQ)
        {
            first_image_time = dStampSec;
            pub_count = 0;
        }
    }
    else
    {
        PUB_THIS_FRAME = false;
    }

    if (PUB_THIS_FRAME)
    {
        pub_count++;
        shared_ptr<IMG_MSG> feature_points(new IMG_MSG());
        feature_points->header = dStampSec;
        vector<set<int>> hash_ids(NUM_OF_CAM);
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            for(unsigned int i=0;i<fts.size(); i++) {
                auto ft = fts[i];
                if(! CompareDoubles2(ft.ts, dStampSec) ) {
                    cerr  << std::setprecision(10)<< " ft ts is : " << ft.ts << " camera total ts: " << dStampSec << std::endl;
                }
                int id_of_point = i;
                // the points here should be vector of image plane coordinates.
                // TODO , change this point from world frame coordinate to image plane coordinate.
                //  space to image plane.
                Eigen::Vector3d pw(ft.point.x(), ft.point.y(), ft.point.z());
                Eigen::Vector2d image_coor ;
                auto res = spaceToPlane(pw, image_coor, params_->fx, params_->fy, params_->cx, params_->cy);
                if (!res) {
                    std::cerr << "image coor is has zero depth, ignore it " << std::endl;
                    //continue;
                }
                // points here is normalized plane (x,y,1)
                feature_points->points.push_back(Vector3d(ft.ft(0), ft.ft(1), 1));
                //feature_points->points.push_back(Vector3d(ft.point.x(), ft.point.y(), ft.point.z()));
                feature_points->id_of_point.push_back(id_of_point);
                feature_points->u_of_point.push_back(image_coor(0));
                feature_points->v_of_point.push_back(image_coor(1));
                // ignore the velocity in pixel frame.(vx,vy)
                feature_points->velocity_x_of_point.push_back(0);
                feature_points->velocity_y_of_point.push_back(0);

            }
            /*auto &un_pts = trackerData[i].cur_un_pts;
            auto &cur_pts = trackerData[i].cur_pts;
            auto &ids = trackerData[i].ids;
            auto &pts_velocity = trackerData[i].pts_velocity;
            for (unsigned int j = 0; j < ids.size(); j++)
            {
                if (trackerData[i].track_cnt[j] > 1)
                {
                    int p_id = ids[j];
                    hash_ids[i].insert(p_id);
                    double x = un_pts[j].x;
                    double y = un_pts[j].y;
                    double z = 1;
                    feature_points->points.push_back(Vector3d(x, y, z));
                    feature_points->id_of_point.push_back(p_id * NUM_OF_CAM + i);
                    feature_points->u_of_point.push_back(cur_pts[j].x);
                    feature_points->v_of_point.push_back(cur_pts[j].y);
                    feature_points->velocity_x_of_point.push_back(pts_velocity[j].x);
                    feature_points->velocity_y_of_point.push_back(pts_velocity[j].y);
                }
            }*/

            //}
            // skip the first image; since no optical speed on frist image
            if (!init_pub)
            {
                cout << std::this_thread::get_id() << std::this_thread::get_id() << "4 PubImage init_pub skip the first image!" << endl;
                init_pub = 1;
            }
            else
            {
                m_buf.lock();
                cout << std::this_thread::get_id() << std::this_thread::get_id() << "publish image fts for camera @ts: " << feature_points->header << std::endl;
                feature_buf.push(feature_points);
                cout << "feature points pushed: " << std::endl;
                for(int i=0;i<feature_points->points.size();i++) {
                    cout << "point xyz: " << feature_points->points[i].x() << ";" <<  feature_points->points[i].y() << ";"
                         << feature_points->points[i].z() << std::endl;
                }

                cout << std::endl;
                
                // cout << std::this_thread::get_id() << std::this_thread::get_id() << "5 PubImage t : " << fixed << feature_points->header
                //     << " feature_buf size: " << feature_buf.size() << endl;
                m_buf.unlock();
                con.notify_one();
            }
        }
    }

}


void System::PubImageData(double dStampSec, Mat &img)
{
    if (!init_feature)
    {
        cout << std::this_thread::get_id() << std::this_thread::get_id() << "1 PubImageData skip the first detected feature, which doesn't contain optical flow speed" << endl;
        init_feature = 1;
        return;
    }

    if (first_image_flag)
    {
        cout << std::this_thread::get_id() << "2 PubImageData first_image_flag" << endl;
        first_image_flag = false;
        first_image_time = dStampSec;
        last_image_time = dStampSec;
        return;
    }
    // detect unstable camera stream
    if (dStampSec - last_image_time > 1.0 || dStampSec < last_image_time)
    {
        cerr << "3 PubImageData image discontinue! reset the feature tracker!" << endl;
        first_image_flag = true;
        last_image_time = 0;
        pub_count = 1;
        return;
    }
    last_image_time = dStampSec;
    // frequency control
    if (round(1.0 * pub_count / (dStampSec - first_image_time)) <= FREQ)
    {
        PUB_THIS_FRAME = true;
        // reset the frequency control
        if (abs(1.0 * pub_count / (dStampSec - first_image_time) - FREQ) < 0.01 * FREQ)
        {
            first_image_time = dStampSec;
            pub_count = 0;
        }
    }
    else
    {
        PUB_THIS_FRAME = false;
    }

    TicToc t_r;
    // cout << std::this_thread::get_id() << "3 PubImageData t : " << dStampSec << endl;
    trackerData[0].readImage(img, dStampSec);

    for (unsigned int i = 0;; i++)
    {
        bool completed = false;
        completed |= trackerData[0].updateID(i);

        if (!completed)
            break;
    }
    if (PUB_THIS_FRAME)
    {
        pub_count++;
        shared_ptr<IMG_MSG> feature_points(new IMG_MSG());
        feature_points->header = dStampSec;
        vector<set<int>> hash_ids(NUM_OF_CAM);
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            auto &un_pts = trackerData[i].cur_un_pts;
            auto &cur_pts = trackerData[i].cur_pts;
            auto &ids = trackerData[i].ids;
            auto &pts_velocity = trackerData[i].pts_velocity;
            for (unsigned int j = 0; j < ids.size(); j++)
            {
                if (trackerData[i].track_cnt[j] > 1)
                {
                    int p_id = ids[j];
                    hash_ids[i].insert(p_id);
                    double x = un_pts[j].x;
                    double y = un_pts[j].y;
                    double z = 1;
                    feature_points->points.push_back(Vector3d(x, y, z));
                    feature_points->id_of_point.push_back(p_id * NUM_OF_CAM + i);
                    feature_points->u_of_point.push_back(cur_pts[j].x);
                    feature_points->v_of_point.push_back(cur_pts[j].y);
                    feature_points->velocity_x_of_point.push_back(pts_velocity[j].x);
                    feature_points->velocity_y_of_point.push_back(pts_velocity[j].y);
                }
            }
            //}
            // skip the first image; since no optical speed on frist image
            if (!init_pub)
            {
                cout << std::this_thread::get_id() << "4 PubImage init_pub skip the first image!" << endl;
                init_pub = 1;
            }
            else
            {
                m_buf.lock();
                
                feature_buf.push(feature_points);
                // cout << std::this_thread::get_id() << "5 PubImage t : " << fixed << feature_points->header
                //     << " feature_buf size: " << feature_buf.size() << endl;
                m_buf.unlock();
                con.notify_one();
            }
        }
    }

#ifdef __linux__
    cv::Mat show_img;
	cv::cvtColor(img, show_img, CV_GRAY2RGB);
	if (SHOW_TRACK)
	{
		for (unsigned int j = 0; j < trackerData[0].cur_pts.size(); j++)
        {
			double len = min(1.0, 1.0 * trackerData[0].track_cnt[j] / WINDOW_SIZE);
			cv::circle(show_img, trackerData[0].cur_pts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
		}

        cv::namedWindow("IMAGE", CV_WINDOW_AUTOSIZE);
		cv::imshow("IMAGE", show_img);
        cv::waitKey(1);
	}
#endif    
    // cout << std::this_thread::get_id() << "5 PubImage" << endl;
    
}

vector<pair<vector<ImuConstPtr>, ImgConstPtr>> System::getMeasurements()
{
    vector<pair<vector<ImuConstPtr>, ImgConstPtr>> measurements;

    while (true)
    {
        if (imu_buf.empty() || feature_buf.empty())
        {
            cerr << std::this_thread::get_id() << "1 imu_buf.empty() || feature_buf.empty()" << endl;
            return measurements;
        }
        // if newest imu is older than that of oldest feature ts.
        if (!(imu_buf.back()->header > feature_buf.front()->header + estimator.td))
        {
            cerr << "wait for imu, only should happen at the beginning sum_of_wait: " 
                << sum_of_wait 
                << " feature_buf.front()  ts: " << feature_buf.front()->header << " td of estimator: " << estimator.td
                 << " measurements size: " << measurements.size() 
                 << "imu_buf back() ts: " << imu_buf.back()->header
                << endl;
            sum_of_wait++;
            return measurements;
        }

        if (!(imu_buf.front()->header < feature_buf.front()->header + estimator.td))
        {
            cerr << "throw img, only should happen at the beginning" << endl;
            feature_buf.pop();
            continue;
        }
        ImgConstPtr img_msg = feature_buf.front();
        feature_buf.pop();

        vector<ImuConstPtr> IMUs;
        while (imu_buf.front()->header < img_msg->header + estimator.td)
        {
            //cout << std::this_thread::get_id() << "imu_buf ts less than img ts , add to buf imus for backend int."  
            //    << "imu buf ts: " << imu_buf.front()->header 
            //    << "img_msg header ts: " << img_msg->header 
            //    << "estimator.id is: " << estimator.td
            //    << std::endl;
            IMUs.emplace_back(imu_buf.front());
            imu_buf.pop();
        }
        // cout << std::this_thread::get_id() << "1 getMeasurements IMUs size: " << IMUs.size() << endl;
        IMUs.emplace_back(imu_buf.front());
        if (IMUs.empty()){
            cerr << "no imu between two image" << endl;
        }
        // cout << std::this_thread::get_id() << "1 getMeasurements img t: " << fixed << img_msg->header
        //     << " imu begin: "<< IMUs.front()->header 
        //     << " end: " << IMUs.back()->header
        //     << endl;
        // out << "has some measurements  with current size of measurements: " << measurements.size() << std::endl;
        measurements.emplace_back(IMUs, img_msg);
        cout << std::this_thread::get_id() << "has some measurements  with current size of measurements: " << measurements.size() << std::endl;
    }
    cout << std::this_thread::get_id() << "final measurements size: " << measurements.size() << std::endl;
    //exit(0);
    return measurements;
}

void System::PubImuData(double dStampSec, const Eigen::Vector3d &vGyr, 
    const Eigen::Vector3d &vAcc)
{
    shared_ptr<IMU_MSG> imu_msg(new IMU_MSG());
	imu_msg->header = dStampSec;
	imu_msg->linear_acceleration = vAcc;
	imu_msg->angular_velocity = vGyr;

    if (dStampSec <= last_imu_t)
    {
        cerr << "imu message in disorder!" << endl;
        return;
    }
    last_imu_t = dStampSec;
    // cout << std::this_thread::get_id() << "1 PubImuData t: " << fixed << imu_msg->header
    //     << " acc: " << imu_msg->linear_acceleration.transpose()
    //     << " gyr: " << imu_msg->angular_velocity.transpose() << endl;
    m_buf.lock();
    imu_buf.push(imu_msg);
    // cout << std::this_thread::get_id() << "1 PubImuData t: " << fixed << imu_msg->header 
    //     << " imu_buf size:" << imu_buf.size() << endl;
    m_buf.unlock();
    con.notify_one();
}

// thread: visual-inertial odometry
void System::ProcessBackEnd()
{
    cout << std::this_thread::get_id() << "1 ProcessBackEnd start" << endl;
    while (bStart_backend)
    {
        // cout << std::this_thread::get_id() << "1 process()" << endl;
        vector<pair<vector<ImuConstPtr>, ImgConstPtr>> measurements;
        
        unique_lock<mutex> lk(m_buf);
        con.wait(lk, [&] {
            return (measurements = getMeasurements()).size() != 0;
        });
        if( measurements.size() > 1){
        cout << std::this_thread::get_id() << "1 getMeasurements size: " << measurements.size() 
            << " imu sizes: " << measurements[0].first.size()
            << " feature_buf size: " <<  feature_buf.size()
            << " imu_buf size: " << imu_buf.size() << endl;
        }
        lk.unlock();
        m_estimator.lock();
        for (auto &measurement : measurements)
        {
            auto img_msg = measurement.second;
            double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0;
            for (auto &imu_msg : measurement.first)
            {
                double t = imu_msg->header;
                double img_t = img_msg->header + estimator.td;
                if (t <= img_t)
                {
                    if (current_time < 0)
                        current_time = t;
                    double dt = t - current_time;
                    assert(dt >= 0);
                    current_time = t;
                    dx = imu_msg->linear_acceleration.x();
                    dy = imu_msg->linear_acceleration.y();
                    dz = imu_msg->linear_acceleration.z();
                    rx = imu_msg->angular_velocity.x();
                    ry = imu_msg->angular_velocity.y();
                    rz = imu_msg->angular_velocity.z();
                    estimator.processIMU(dt, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                    // printf("1 BackEnd imu: dt:%f a: %f %f %f w: %f %f %f\n",dt, dx, dy, dz, rx, ry, rz);
                }
                else
                {
                    double dt_1 = img_t - current_time;
                    double dt_2 = t - img_t;
                    current_time = img_t;
                    assert(dt_1 >= 0);
                    assert(dt_2 >= 0);
                    assert(dt_1 + dt_2 > 0);
                    double w1 = dt_2 / (dt_1 + dt_2);
                    double w2 = dt_1 / (dt_1 + dt_2);
                    dx = w1 * dx + w2 * imu_msg->linear_acceleration.x();
                    dy = w1 * dy + w2 * imu_msg->linear_acceleration.y();
                    dz = w1 * dz + w2 * imu_msg->linear_acceleration.z();
                    rx = w1 * rx + w2 * imu_msg->angular_velocity.x();
                    ry = w1 * ry + w2 * imu_msg->angular_velocity.y();
                    rz = w1 * rz + w2 * imu_msg->angular_velocity.z();
                    estimator.processIMU(dt_1, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                    //printf("dimu: dt:%f a: %f %f %f w: %f %f %f\n",dt_1, dx, dy, dz, rx, ry, rz);
                }
            }

            // cout << std::this_thread::get_id() << "processing vision data with stamp:" << img_msg->header 
            //     << " img_msg->points.size: "<< img_msg->points.size() << endl;

            // TicToc t_s;
            map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> image;
            for (unsigned int i = 0; i < img_msg->points.size(); i++) 
            {
                int v = img_msg->id_of_point[i] + 0.5;
                int feature_id = v / NUM_OF_CAM;
                int camera_id = v % NUM_OF_CAM;
                double x = img_msg->points[i].x();
                double y = img_msg->points[i].y();
                double z = img_msg->points[i].z();
                double p_u = img_msg->u_of_point[i];
                double p_v = img_msg->v_of_point[i];
                double velocity_x = img_msg->velocity_x_of_point[i];
                double velocity_y = img_msg->velocity_y_of_point[i];
                assert(z == 1);
                Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
                xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
                image[feature_id].emplace_back(camera_id, xyz_uv_velocity);
            }
            TicToc t_processImage;
            estimator.processImage(image, img_msg->header);
            
            if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
            {
                Vector3d p_wi;
                Quaterniond q_wi;
                q_wi = Quaterniond(estimator.Rs[WINDOW_SIZE]);
                p_wi = estimator.Ps[WINDOW_SIZE];
                vPath_to_draw.push_back(p_wi);
                double dStamp = estimator.Headers[WINDOW_SIZE];
                cout << std::this_thread::get_id() << "1 BackEnd processImage dt: " << fixed << t_processImage.toc() << " stamp: " <<  dStamp << " p_wi: " << p_wi.transpose() << endl;
                ofs_pose << fixed << dStamp << " " << p_wi(0) << " " << p_wi(1) << " " << p_wi(2) << " " 
                         << q_wi.w() << " " << q_wi.x() << " " << q_wi.y() << " " << q_wi.z() << endl;
            }
        }
        m_estimator.unlock();
    }
}

void System::Draw() 
{   
    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    s_cam = pangolin::OpenGlRenderState(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 384, 0.1, 1000),
            pangolin::ModelViewLookAt(-5, 0, 15, 7, 0, 0, 1.0, 0.0, 0.0)
    );

    d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

    // pangolin::OpenGlRenderState s_cam(
    //         pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 384, 0.1, 1000),
    //         pangolin::ModelViewLookAt(-5, 0, 15, 7, 0, 0, 1.0, 0.0, 0.0)
    // );

    // pangolin::View &d_cam = pangolin::CreateDisplay()
    //         .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
    //         .SetHandler(new pangolin::Handler3D(s_cam));

    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(0.75f, 0.75f, 0.75f, 0.75f);
        glColor3f(0, 0, 1);
        pangolin::glDrawAxis(3);
         
        // draw poses
        glColor3f(0, 0, 0);
        glLineWidth(2);
        glBegin(GL_LINES);
        int nPath_size = vPath_to_draw.size();
        for(int i = 0; i < nPath_size-1; ++i)
        {        
            glVertex3f(vPath_to_draw[i].x(), vPath_to_draw[i].y(), vPath_to_draw[i].z());
            glVertex3f(vPath_to_draw[i+1].x(), vPath_to_draw[i+1].y(), vPath_to_draw[i+1].z());
        }
        glEnd();
        
        // points
        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
        {
            glPointSize(5);
            glBegin(GL_POINTS);
            for(int i = 0; i < WINDOW_SIZE+1;++i)
            {
                Vector3d p_wi = estimator.Ps[i];
                glColor3f(1, 0, 0);
                glVertex3d(p_wi[0],p_wi[1],p_wi[2]);
            }
            glEnd();
        }
        if (!pangolin::ShouldQuit())
            pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
}

#ifdef __APPLE__
void System::InitDrawGL() 
{   
    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    s_cam = pangolin::OpenGlRenderState(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 384, 0.1, 1000),
            pangolin::ModelViewLookAt(-5, 0, 15, 7, 0, 0, 1.0, 0.0, 0.0)
    );

    d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));
}

void System::DrawGLFrame() 
{  

    if (pangolin::ShouldQuit() == false)
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(0.75f, 0.75f, 0.75f, 0.75f);
        glColor3f(0, 0, 1);
        pangolin::glDrawAxis(3);
            
        // draw poses
        glColor3f(0, 0, 0);
        glLineWidth(2);
        glBegin(GL_LINES);
        int nPath_size = vPath_to_draw.size();
        for(int i = 0; i < nPath_size-1; ++i)
        {        
            glVertex3f(vPath_to_draw[i].x(), vPath_to_draw[i].y(), vPath_to_draw[i].z());
            glVertex3f(vPath_to_draw[i+1].x(), vPath_to_draw[i+1].y(), vPath_to_draw[i+1].z());
        }
        glEnd();
        
        // points
        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
        {
            glPointSize(5);
            glBegin(GL_POINTS);
            for(int i = 0; i < WINDOW_SIZE+1;++i)
            {
                Vector3d p_wi = estimator.Ps[i];
                glColor3f(1, 0, 0);
                glVertex3d(p_wi[0],p_wi[1],p_wi[2]);
            }
            glEnd();
        }
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
}


#endif
} // namespace sim


