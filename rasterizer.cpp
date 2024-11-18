#define NOMINMAX
#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>
#include <iostream>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


static bool insideTriangle(float x, float y, const Vector3f* _v)
{   
    Vector3f Q = {float(x), float(y), 0};

    Vector3f p0p1 = _v[1] - _v[0];
    Vector3f p0Q = Q - _v[0];

    Vector3f p1p2 = _v[2] - _v[1];
    Vector3f p1Q = Q - _v[1];

    Vector3f p2p0 = _v[0] - _v[2];
    Vector3f p2Q = Q - _v[2];

    //The class definition is already counterclockwise, so we only have to consider the same positive case
    return p0p1.cross(p0Q).z() > 0 && p1p2.cross(p1Q).z() > 0 && p2p0.cross(p2Q).z()>0;
    
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = -vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
}



/* 
����4�Ĵ��룬δ����SSAA 
*/

//Screen space rasterization
//void rst::rasterizer::rasterize_triangle(const Triangle& t) {
//    auto v = t.toVector4();
//    // ��ȡ�����εİ�Χ��
//    float tmp = 0;
//    tmp = std::min(v[0].x(), v[1].x());
//    tmp = std::min(tmp, v[2].x());
//    int min_x = std::max((float)0, tmp);
//    tmp = std::max(v[0].x(), v[1].x());
//    tmp = std::max(tmp, v[2].x());
//    int max_x = std::min(static_cast<float>(width) - 1, tmp);
//
//    tmp = std::min(v[0].y(), v[1].y());
//    tmp = std::min(tmp, v[2].y());
//    int min_y = std::max((float)0, tmp);
//    tmp = std::max(v[0].y(), v[1].y());
//    tmp = std::max(tmp, v[2].y());
//    int max_y = std::min(static_cast<float>(height) - 1, tmp);
//
//    // ������Χ�е�ÿһ�����ص�
//    for (int x = min_x; x <= max_x; x++)
//        for (int y = min_y; y <= max_y; y++) {
//
//            // ����õ����������ڣ�˵�����Խ�����Ⱦ
//            if (insideTriangle(x + 0.5, y + 0.5, t.v)) {
//
//                // �������������ֵ������õ�����ֵ
//                auto [alpha, beta, gamma] = computeBarycentric2D(x + 0.5 , y + 0.5, t.v);   // ������������
//                float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w()); // �������ĵĹ�һ������
//                float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();// �����ֵ���ֵ
//                z_interpolated *= w_reciprocal; //��һ������
//
//                // ��ȡ��Ȼ��������
//                int idx = get_index(x, y);
//
//                // ȷ���õ����С�ڵ�ǰ��Ȼ���Ż���Ⱦ
//                if (z_interpolated < depth_buf[idx]) {
//                    set_pixel(Eigen::Vector3f(x, y, z_interpolated), t.getColor()); // ��Ⱦ�����ص�
//                    depth_buf[idx] = z_interpolated;    // ���µ�ǰ��Ȼ���
//                }
//            }
//        }
//
//}



/*
����5�Ĵ��룬����SSAA
*/
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();
    // ��ȡ�����εİ�Χ��
    float tmp = 0;
    tmp = std::min(v[0].x(), v[1].x());
    tmp = std::min(tmp, v[2].x());
    int min_x = std::max((float)0, tmp);
    tmp = std::max(v[0].x(), v[1].x());
    tmp = std::max(tmp, v[2].x());
    int max_x = std::min(static_cast<float>(width) - 1, tmp);

    tmp = std::min(v[0].y(), v[1].y());
    tmp = std::min(tmp, v[2].y());
    int min_y = std::max((float)0, tmp);
    tmp = std::max(v[0].y(), v[1].y());
    tmp = std::max(tmp, v[2].y());
    int max_y = std::min(static_cast<float>(height) - 1, tmp);
    
    // ������Χ�е�ÿһ�����ص�
    for(int x = min_x;x<=max_x;x++)
        for (int y = min_y; y <= max_y; y++) {
            // �����ص��ڷֳɵ�4����������д�����������
            // dx��dy��ƫ������ӳ���ϵΪdx = 0 -> x' = x + 0.25 ,dx = 1 -> x' = x + 0.75
            // x'���������x����,yͬ��
            for (double dx = 0; dx < 2 ; dx += 1)
                for (double dy = 0; dy < 2; dy += 1) {
                    // �ж��������Ƿ�����������
                    // ����x + 0.5 * dx + 0.25��y + 0.5 * dy + 0.25�Ǽ��������������
                    if (insideTriangle(x + 0.5 * dx + 0.25, y + 0.5 * dy + 0.25, t.v)) {
                        // ��ֵ�������
                        auto [alpha, beta, gamma] = computeBarycentric2D(x + 0.5 * dx + 0.25, y + 0.5 * dy + 0.25, t.v);
                        float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                        float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                        z_interpolated *= w_reciprocal;
                        // ��ȡSSAAר�õ���Ȼ�����������
                        int idx = get_super_index(x + 0.5 * dx + 0.25, y + 0.5 * dy + 0.25);
                        if (z_interpolated < ssaa_depth_buf[idx]){
                            // Ϊ��������ɫ���Ҹ������ֵ
                            ssaa_set_pixel(Eigen::Vector3f(x + 0.5 * dx + 0.25, y + 0.5 * dy + 0.25, z_interpolated), t.getColor());
                            ssaa_depth_buf[idx] = z_interpolated;
                        }
                    }
                }
            // 4�������������Ϻ󣬶����������ص��������ɫȡ4���������ƽ��
            Eigen::Vector3f color = { 0,0,0 };
            color += ssaa_frame_buf[get_super_index(x + 0.25, y + 0.25)];
            color += ssaa_frame_buf[get_super_index(x + 0.75, y + 0.25)];
            color += ssaa_frame_buf[get_super_index(x + 0.25, y + 0.75)];
            color += ssaa_frame_buf[get_super_index(x + 0.75, y + 0.75)];
            set_pixel(Eigen::Vector3f(x, y, 1), color/4);
        }
        
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
        std::fill(ssaa_frame_buf.begin(), ssaa_frame_buf.end(), Eigen::Vector3f{ 0, 0, 0 });
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
        std::fill(ssaa_depth_buf.begin(), ssaa_depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
    ssaa_frame_buf.resize(w * h * 4);
    ssaa_depth_buf.resize(w * h * 4);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

int rst::rasterizer::get_super_index(double x, double y)
{
    int new_x = (x - 0.25) * 2;
    int new_y = (y - 0.25) * 2;
    return (height*2 - 1 - new_y) * width*2 + new_x;
}



void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

void rst::rasterizer::ssaa_set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    int new_x = (point.x() - 0.25) * 2;
    int new_y = (point.y() - 0.25) * 2;

    auto ind = (height * 2 - 1 - new_y) * width * 2 + new_x;
    ssaa_frame_buf[ind] = color;

}

// clang-format on