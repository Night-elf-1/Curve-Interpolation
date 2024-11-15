#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;
using namespace std;

struct State {
    double x;
    double y;
    double vx;
    double vy;
    double ax;
    double ay;
};

int main() {
    // 定义初始和终止时间
    double t0 = 0.0;
    double t1 = 17.07;

    // 初始和终止状态
    Eigen::MatrixXd state_t0(3, 2);
    state_t0 << 0, 0,           // x y
                0, 0,           // vx vy
                0, 0;           // ax ay

    Eigen::MatrixXd state_t1(3, 2);
    state_t1 << 6, 0,
                0, 0,
                0, 0;

    // 计算 A 和 B 系数矩阵
    Eigen::VectorXd X(6), Y(6);         // 创建一维的矩阵，用于存储状态 起点和终点的 x y vx vy ax ay
    X << state_t0(0, 0), state_t0(1, 0), state_t0(2, 0), state_t1(0, 0), state_t1(1, 0), state_t1(2, 0);        // x 方向
    Y << state_t0(0, 1), state_t0(1, 1), state_t0(2, 1), state_t1(0, 1), state_t1(1, 1), state_t1(2, 1);

    Eigen::MatrixXd T(6, 6);
    T << std::pow(t0, 5), std::pow(t0, 4), std::pow(t0, 3), std::pow(t0, 2), t0, 1,
         5 * std::pow(t0, 4), 4 * std::pow(t0, 3), 3 * std::pow(t0, 2), 2 * t0, 1, 0,
         20 * std::pow(t0, 3), 12 * std::pow(t0, 2), 6 * t0, 1, 0, 0,
         std::pow(t1, 5), std::pow(t1, 4), std::pow(t1, 3), std::pow(t1, 2), t1, 1,
         5 * std::pow(t1, 4), 4 * std::pow(t1, 3), 3 * std::pow(t1, 2), 2 * t1, 1, 0,
         20 * std::pow(t1, 3), 12 * std::pow(t1, 2), 6 * t1, 1, 0, 0;

    Eigen::VectorXd A = T.inverse() * X;
    Eigen::VectorXd B = T.inverse() * Y;

    // 离散化时间
    double dt = 0.001;
    std::vector<State> path;
    std::vector<double> x_;
    for (double t = t0; t <= t1; t += dt) {
        Eigen::VectorXd T_poly(6);
        T_poly << std::pow(t, 5), std::pow(t, 4), std::pow(t, 3), std::pow(t, 2), t, 1;

        Eigen::VectorXd T_poly_deriv(6);
        T_poly_deriv << 5 * std::pow(t, 4), 4 * std::pow(t, 3), 3 * std::pow(t, 2), 2 * t, 1, 0;

        // 加速度项的多项式系数（二阶导数）
        Eigen::VectorXd T_poly_deriv2(6);
        T_poly_deriv2 << 20 * std::pow(t, 3), 12 * std::pow(t, 2), 6 * t, 2, 0, 0;

        State state;
        state.x = T_poly.dot(A);               // 纵向坐标
        state.y = T_poly.dot(B);               // 横向坐标
        state.vx = T_poly_deriv.dot(A);        // 纵向速度
        state.vy = T_poly_deriv.dot(B);        // 横向速度
        state.ax = T_poly_deriv2.dot(A);       // 纵向加速度
        state.ay = T_poly_deriv2.dot(B);       // 横向加速度

        path.push_back(state);
        x_.push_back(t);
    }

    std::vector<double> velocity_x;
    std::vector<double> velocity_y;
    std::vector<double> position_x;
    std::vector<double> position_y;
    std::vector<double> accel_x;
    std::vector<double> accel_y;
    // 打印路径数据
    for (const auto& p : path) {
        std::cout << "x: " << p.x << ", y: " << p.y << ", vx: " << p.vx << ", vy: " << p.vy << std::endl;
        velocity_x.push_back(p.vx);
        velocity_y.push_back(p.vy);
        position_x.push_back(p.x);
        position_y.push_back(p.y);
        accel_x.push_back(p.ax);
        accel_y.push_back(p.ay);
    }
    
    plt::figure_size(800, 600);
    plt::plot(x_, velocity_x, "r-");
    plt::plot(x_, accel_x, "b-");
    plt::legend();
    plt::title("Velocity and Acceleration Profile");
    plt::xlabel("Time (s)");
    plt::ylabel("Value");

    plt::figure_size(800, 600);
    plt::plot(position_x, position_y, "g-");
    plt::title("Path Profile");
    plt::xlabel("X Position");
    plt::ylabel("Y Position");

    plt::show();

    return 0;
}
