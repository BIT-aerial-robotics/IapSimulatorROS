// Callback.h
// callback functions
// Du Jianrui   2023.1.31

#pragma once

#include "std_msgs/Float64.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PointStamped.h"

class Thrust
{
public:
    void callback(const std_msgs::Float64::ConstPtr& _msg)
    {
        thrust_ = *_msg;
    }

    std_msgs::Float64 get() const
    {
        return thrust_;
    }

private:
    std_msgs::Float64 thrust_;
};

class EulerAngle
{
public:
    void callback(const geometry_msgs::Point::ConstPtr& _msg)
    {
        eulerAngle_ = *_msg;
    }

    geometry_msgs::Point get() const
    {
        return eulerAngle_;
    }

private:
    geometry_msgs::Point eulerAngle_;
};

class Position
{
public:
    void callback(const geometry_msgs::Point::ConstPtr& _msg)
    {
        position_ = *_msg;
    }

    geometry_msgs::Point get() const
    {
        return position_;
    }

private:
    geometry_msgs::Point position_;
};

class BodyRate
{
public:
    void callback(const geometry_msgs::Point::ConstPtr& _msg)
    {
        bodyRate_ = *_msg;
    }

    geometry_msgs::Point get() const
    {
        return bodyRate_;
    }

private:
    geometry_msgs::Point bodyRate_;
};

class Velocity
{
public:
    void callback(const geometry_msgs::Point::ConstPtr& _msg)
    {
        velocity_ = *_msg;
    }

    geometry_msgs::Point get() const
    {
        return velocity_;
    }

private:
    geometry_msgs::Point velocity_;
};

class AngularAcceleration
{
public:
    void callback(const geometry_msgs::Point::ConstPtr& _msg)
    {
        angularAcceleration_ = *_msg;
    }

    geometry_msgs::Point get() const
    {
        return angularAcceleration_;
    }

private:
    geometry_msgs::Point angularAcceleration_;
};

class LinearAcceleration
{
public:
    void callback(const geometry_msgs::Point::ConstPtr& _msg)
    {
        linearAcceleration_ = *_msg;
    }

    geometry_msgs::Point get() const
    {
        return linearAcceleration_;
    }

private:
    geometry_msgs::Point linearAcceleration_;
};

class Force
{
public:
    void callback(const geometry_msgs::Point::ConstPtr& _msg)
    {
        force_ = *_msg;
    }

    geometry_msgs::Point get() const
    {
        return force_;
    }

private:
    geometry_msgs::Point force_;
};

class Moment
{
public:
    void callback(const geometry_msgs::Point::ConstPtr& _msg)
    {
        moment_ = *_msg;
    }

    geometry_msgs::Point get() const
    {
        return moment_;
    }

private:
    geometry_msgs::Point moment_;
};

class Time
{
public:
    void callback(const geometry_msgs::PointStamped::ConstPtr& _msg)
    {
        moment_ = *_msg;
    }

    geometry_msgs::PointStamped get() const
    {
        return moment_;
    }

private:
    geometry_msgs::PointStamped moment_;
};
