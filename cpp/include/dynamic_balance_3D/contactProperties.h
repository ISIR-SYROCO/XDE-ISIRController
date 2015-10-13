/**
 * \file ContactProp.h
 * \author Darwin Lau
 *
 * \brief Dimensions of the contact
 */
#ifndef CONTACTPROP_H
#define CONTACTPROP_H

#include <Eigen/Dense>
#include <Eigen/Lgsm>

class ContactProperties 
{
    public:
        ContactProperties(double t, double b, double l, double r);
        /*
                    ^ Y
                    |
               tl---|---tr
                |   |   |
                |   +---|----> X
                |       |
               bl-------br
        */
        double top;
        double bottom;
        double left;
        double right;

        Eigen::Vector3d pTopRight() const { return Eigen::Vector3d(right, top, 0.0); };
        Eigen::Vector3d pTopLeft() const { return Eigen::Vector3d(left, top, 0.0); };
        Eigen::Vector3d pBottomRight() const { return Eigen::Vector3d(right, bottom, 0.0); };
        Eigen::Vector3d pBottomLeft() const { return Eigen::Vector3d(left, bottom, 0.0); };
        Eigen::Vector3d pTop() const { return Eigen::Vector3d(0.0, top, 0.0); };
        Eigen::Vector3d pBottom() const { return Eigen::Vector3d(0.0, bottom, 0.0); };
};

#endif
