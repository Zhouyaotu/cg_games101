#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_rotation(Vector3f axis, float angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    Eigen::Matrix4f M_rotation; 
    Eigen::Matrix4f M_trans;
    Eigen::Vector3f u,v,w, temp;
    float radian_angle = angle/180.0f*MY_PI;
    w = axis/axis.norm();
    temp << 1.0, 1.0, 0.0;
    u = temp.cross(w);
    u /= u.norm();
    v = w.cross(u);
    
    M_trans <<   u.x(), v.x(), w.x(), 0, 
                 u.y(), v.y(), w.y(), 0, 
                 u.z(), v.z(), w.z(), 0, 
                 0, 0, 0, 1;
    
    //std::cout<<"1\n";
    M_rotation << cos(radian_angle), -sin(radian_angle), 0, 0, 
                  sin(radian_angle), cos(radian_angle), 0, 0, 
                  0, 0, 1, 0, 
                  0, 0, 0, 1;
    //std::cout<<"2\n";
    std::cout<<M_trans<<"\n";
    model = M_trans * M_rotation * M_trans.transpose() * model;
    //model = M_rotation * M_trans * model;
    //std::cout<<"3\n";
    //std::cout<<M_trans.transpose()<<"\n";
    return model;
}

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate; //应该是假设相机的`方向`与世界坐标系重合，即看向-z轴方向
    translate << 1, 0, 0, -eye_pos[0], 
                 0, 1, 0, -eye_pos[1], 
                 0, 0, 1, -eye_pos[2], 
                 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    Eigen::Matrix4f translate; 
    translate << cos(rotation_angle), -sin(rotation_angle), 0, 0, 
                 sin(rotation_angle), cos(rotation_angle), 0, 0, 
                 0, 0, 1, 0, 
                 0, 0, 0, 1;
    model = translate * model;
    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
    float l,r,t,b;
    t = zNear * tan(eye_fov/180*MY_PI/2);
    r = t * aspect_ratio;
    l = -r;
    b = -t;

    Eigen::Matrix4f M_orth, M_persp;

    M_orth << 2/(r-l), 0, 0, -(r+l)/(r-l), 
                 0, 2/(t-b), 0, -(t+b)/(t-b), 
                 0, 0, 2/(zNear-zFar), -(zNear+zFar)/(zNear-zFar), 
                 0, 0, 0, 1;

    M_persp << zNear, 0, 0, 0, 
                 0, zNear, 0, 0, 
                 0, 0, zFar+zNear, -zFar*zNear, 
                 0, 0, -1, 0;

    projection = M_orth * M_persp * projection; 
    return projection;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f rotationAxis = {5, 15, 1};

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -5}, {0, 2, -5}, {-2, 0, -5}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        //r.set_model(get_model_matrix(angle));
        r.set_model(get_rotation(rotationAxis, angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        //r.set_model(get_model_matrix(angle));
        //std::cout<<"1\n";
        r.set_model(get_rotation(rotationAxis, angle));
        //std::cout<<"2\n";
        r.set_view(get_view_matrix(eye_pos));
        //std::cout<<"3\n";
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));
        //std::cout<<"4\n";
        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        //std::cout<<"5\n";
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << angle << "\n";
        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
    }

    return 0;
}
