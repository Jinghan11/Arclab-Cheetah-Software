/*
*author:Technician13
*date:2020.4.10
*/
#ifndef QUADRUPEDINVERSEKINEMATIC_H_
#define QUADRUPEDINVERSEKINEMATIC_H_

#include<math.h>


class QuadrupedInverseKinematic
{
    public:
        //List the functions one by one for preserving the interface
        double calc_dyz();
        double calc_lyz();
        float calc_L_gamma();
        float calc_R_gamma();
        double calc_lxz();
        double calc_n();
        float calc_beta();
        float calc_alpha();
        void set_toe_position(double x,double y, double z);
        void set_leg_length(float len1,float len2, float len3);
    private:
        //given         
        double pos_x;//The position of the toe - X axis
        double pos_y;//The position of the toe - Y axis
        double pos_z;//The position of the toe - Z axis
        float h;//The length of the first link
        float hu;//The length of the second link
        float hl;//The length of the third link
        //find
        float L_gamma;//The first joint angle rotation angle of left legs
        float R_gamma;//The first joint angle rotation angle of left legs
        float alpha;//The second joint angle rotation angle
        float beta;//The third joint angle rotation angle
        //middle parameters
        double dyz;//Length from O to P on Y-Z
        double lyz;//Length from top-pitch-joint to P on Y-Z
        double lxz;//Length from Qxz' to Pxz'
        double n;

};

#endif