/**
 * \file ContactProp.cpp
 * \author Darwin Lau
 *
 * \brief Dimensions of the contact
 */
#include "dynamic_balance_3D/contactProperties.h"

ContactProperties::ContactProperties(double t, double b, double l, double r)
    : top(t), bottom(b), left(l), right(r)
{
}
