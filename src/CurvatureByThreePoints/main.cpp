#include <iostream>
#include <vector>
#include <cmath>
#include <numeric> // For accumulate

using namespace std;

// 计算两点之间的距离
double computeDistance(const vector<double>& p1, const vector<double>& p2) {
    return sqrt(pow(p1[0] - p2[0], 2) + pow(p1[1] - p2[1], 2));
}

int main() {
    // 假设 refPath 已经被加载为二维数组
    vector<vector<double>> refPath = {
        {0.0, 0.0},
        {1.0, 1.0},
        {2.0, 0.0},
        {3.0, -1.0},
        {4.0, 0.0}
    }; // 示例数据，替换为你的实际路径数据

    vector<double> curvature;
    size_t n = refPath.size();

    // 基于三点求外接圆的曲率
    for (size_t i = 0; i < n - 2; ++i) {
        vector<double> A = refPath[i];              // 计算的曲率必须前后都有点 A为前一个点
        vector<double> B = refPath[i + 1];          // 计算曲率的点
        vector<double> C = refPath[i + 2];          // 后一个点

        // 计算三条边的长度
        double a = computeDistance(C, B);
        double b = computeDistance(C, A);
        double c = computeDistance(A, B);

        // 计算中间点的角度值B
        double theta_B = acos((a * a + c * c - b * b) / (2 * a * c));
        double cur = 2 * sin(theta_B) / b;          // 曲率计算公式: k = 1 / r = (2 * sinB) / b
                                                    // r = b / 2 * sinB
        curvature.push_back(cur);
    }

    // 计算路径长度
    vector<double> cumLength;
    double cumulativeLength = 0.0;
    cumLength.push_back(cumulativeLength);
    for (size_t i = 1; i < n; ++i) {
        cumulativeLength += computeDistance(refPath[i - 1], refPath[i]);
        cumLength.push_back(cumulativeLength);
    }

    // 打印参考曲线
    cout << "Reference Path:" << endl;
    for (const auto& point : refPath) {
        cout << "(" << point[0] << ", " << point[1] << ")" << endl;
    }

    // 打印曲率
    cout << "Curvature:" << endl;
    for (size_t i = 0; i < curvature.size(); ++i) {
        cout << "s = " << cumLength[i] << ", curvature = " << curvature[i] << endl;
    }

    return 0;
}
