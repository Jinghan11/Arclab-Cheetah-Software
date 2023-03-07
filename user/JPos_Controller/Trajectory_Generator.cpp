#include <iostream>
#include <vector>
#include <cmath>
#include <stdio.h>
using namespace std;

int main()
{
    double lamda = 0.5; // 摆动周期占比
    double Ts = 1; // 周期时间
    double xs = 0; // x 轴起始点
    double xf = 30; // x 轴终点
    double h = 20; // 抬腿高度
    double zs = -50; // z 轴起点

    vector<double> x; // 存储 x 坐标
    vector<double> z; // 存储 z 坐标

    double x_t = 0;
    double z_t = 0;
    double x_last = 0;
    double z_last = 0;
    double t_last = 0;

    for (double t = 0; t <= Ts; t += 0.01)
    {
        if (t >= 0 && t <= lamda * Ts)
        {
            double sigma = 2 * M_PI * t / (lamda * Ts);
            x_t = (xf - xs) * ((sigma - sin(sigma)) / (2 * M_PI)) + xs;
            z_t = h * (1 - cos(sigma)) / 2 + zs;
            x.push_back(x_t);
            z.push_back(z_t);
        }
        else if (t > lamda * Ts && t < Ts)
        {
            x_t = x_last - (x_last - xs) / ((Ts - t) / (t - t_last));
            z_t = z_last;
            x.push_back(x_t);
            z.push_back(z_t);
        }
        x_last = x_t;
        z_last = z_t;
        t_last = t;
    }

    // 输出 x 和 z 坐标
    for (int i = 0; i < x.size(); i++)
    {
        cout << "x[" << i << "] = " << x[i] << ", z[" << i << "] = " << z[i] << endl;
    }

    return 0;
}
