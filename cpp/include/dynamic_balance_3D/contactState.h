/**
 * \file ContactState.h
 * \author Darwin Lau
 *
 * \brief Stores the state of the contact point
 */
#include <Eigen/Dense>
#include <Eigen/Lgsm>
#include "dynamic_balance_3D/contactProperties.h"

#ifndef CONTACTSTATE_H
#define CONTACTSTATE_H

class ContactState 
{
    public:
        ContactState(const ContactProperties& prop);
        // Sets the position (x_C) and orientation (normal vector of constraint). The Z axis is point out of the page and direction of the contact force
        /*
                    ^ Y
                    |
               tl---|---tr
                |   |   |
                |   +---|----> X
                |       |
               bl-------br
        */
        void setState(Eigen::Displacementd d, int inContact);

        Eigen::Vector3d tr() const { return _disp.getRotation() * Eigen::Vector3d(prop.right, prop.top, 0.0); };
        Eigen::Vector3d tl() const { return _disp.getRotation() * Eigen::Vector3d(prop.left, prop.top, 0.0); };
        Eigen::Vector3d br() const { return _disp.getRotation() * Eigen::Vector3d(prop.right, prop.bottom, 0.0); };
        Eigen::Vector3d bl() const { return _disp.getRotation() * Eigen::Vector3d(prop.left, prop.bottom, 0.0); };
        Eigen::Vector3d t() const { return _disp.getRotation() * Eigen::Vector3d(0.0, prop.top, 0.0); };
        Eigen::Vector3d b() const { return _disp.getRotation() * Eigen::Vector3d(0.0, prop.bottom, 0.0); };
        
        Eigen::Vector3d topRight() const { return _disp.getTranslation() + tr(); }; 
        Eigen::Vector3d topLeft() const { return _disp.getTranslation() + tl(); };
        Eigen::Vector3d bottomRight() const { return _disp.getTranslation() + br(); };
        Eigen::Vector3d bottomLeft() const { return _disp.getTranslation() + bl(); };
        Eigen::Vector3d top() const { return _disp.getTranslation() + t(); };
        Eigen::Vector3d bottom() const { return _disp.getTranslation() + b(); };
        Eigen::Vector3d centre() const { return _disp.getTranslation(); };

        int inContact() const { return _inContact; };

        ContactProperties prop;

    private:
        Eigen::Displacementd _disp;
        int _inContact;
};

#endif
