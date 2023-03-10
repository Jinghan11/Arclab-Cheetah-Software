/*
*author:Technician13
*date:2020.4.10
*/
#include"QuadrupedInverseKinematic.h"
#include<iostream>
#include<cmath>

double QuadrupedInverseKinematic::calc_dyz()
{
    dyz = sqrt(pos_y*pos_y+pos_z*pos_z);
    //std::cout<<dyz<<std::endl;
    return dyz;
}

double QuadrupedInverseKinematic::calc_lyz()
{
    lyz = sqrt(dyz*dyz-h*h);
    //std::cout<<lyz<<std::endl;
    return lyz;
}

float QuadrupedInverseKinematic::calc_R_gamma()
{
    float gamma_1, gamma_2;
    gamma_1=atan(pos_y/pos_z);
    gamma_2=atan(h/lyz);
    R_gamma = gamma_1 - gamma_2;
    //std::cout<<R_gamma<<std::endl;
    return R_gamma;
}

float QuadrupedInverseKinematic::calc_L_gamma()
{
    float gamma_1, gamma_2;
    gamma_1=atan(pos_y/pos_z);
    gamma_2=atan(h/lyz);
    L_gamma = gamma_1 + gamma_2;
    //std::cout<<L_gamma<<std::endl;
    return L_gamma;
}

double QuadrupedInverseKinematic::calc_lxz()
{
    lxz = sqrt(lyz*lyz+pos_x*pos_x);
    //std::cout<<lxz<<std::endl;
    return lxz;
}

double QuadrupedInverseKinematic::calc_n()
{
    n = (lxz*lxz-hl*hl-hu*hu)/(2*hu);
    //std::cout<<n<<std::endl;
    return n;
}

float QuadrupedInverseKinematic::calc_beta()
{
    beta = acos(n/hl);
    //std::cout<<beta<<std::endl;
    return beta;
}

float QuadrupedInverseKinematic::calc_alpha()
{
    float alpha_1, alpha_2;
    alpha_1 = atan(pos_x/lyz);
    alpha_2 = -acos((hu+n)/lxz);
    alpha = alpha_1 + alpha_2;
    //std::cout<<alpha<<std::endl;
    return alpha;
}

void QuadrupedInverseKinematic::set_toe_position(double x, double y, double z)
{
    pos_x = x;
    pos_y = y;
    pos_z = z;
}

void QuadrupedInverseKinematic::set_leg_length(float len1,float len2, float len3)
{
    h = len1;
    hu = len2;
    hl = len3;
}