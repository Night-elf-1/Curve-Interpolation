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
};

int main() {
    // 定义初始和终止时间
    double t0 = 0.0;
    double t1 = 5.0;

    // 初始和终止状态
    Eigen::MatrixXd state_t0(3, 2);
    state_t0 << 0, 0,
                0, 0,
                0, 0;

    Eigen::MatrixXd state_t1(3, 2);
    state_t1 << 3, 0,
                0, 0,
                0, 0;

    // 计算 A 和 B 系数矩阵
    Eigen::VectorXd X(6), Y(6);
    X << state_t0(0, 0), state_t0(1, 0), state_t0(2, 0), state_t1(0, 0), state_t1(1, 0), state_t1(2, 0);
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
    double dt = 0.05;
    std::vector<State> path;

    for (double t = t0; t <= t1; t += dt) {
        Eigen::VectorXd T_poly(6);
        T_poly << std::pow(t, 5), std::pow(t, 4), std::pow(t, 3), std::pow(t, 2), t, 1;

        Eigen::VectorXd T_poly_deriv(6);
        T_poly_deriv << 5 * std::pow(t, 4), 4 * std::pow(t, 3), 3 * std::pow(t, 2), 2 * t, 1, 0;

        State state;
        state.x = T_poly.dot(A);               // 纵向坐标
        state.y = T_poly.dot(B);               // 横向坐标
        state.vx = T_poly_deriv.dot(A);        // 纵向速度
        state.vy = T_poly_deriv.dot(B);        // 横向速度

        path.push_back(state);
    }

    std::vector<double> velocity_;
    std::vector<double> x_;
    // 打印路径数据
    for (const auto& p : path) {
        std::cout << "x: " << p.x << ", y: " << p.y << ", vx: " << p.vx << ", vy: " << p.vy << std::endl;
        velocity_.push_back(p.vx);
        x_.push_back(p.x);
    }
    cout << "速度容器数量 = " << velocity_.size() << endl;

    plt::figure_size(800, 600);
    plt::plot(x_, velocity_, "r-"); 
    plt::show();

    return 0;
}
