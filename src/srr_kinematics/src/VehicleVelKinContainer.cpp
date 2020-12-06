#include "srr_kinematics/VehicleVelKinContainer.hpp"

#include "geometry_msgs/Pose2D.h"
#include "srr_msgs/VehicleVelKinSegment.h"

#include <vector>
#include <cmath>

SRR::VehicleVelKinContainer::VehicleVelKinContainer(double _theta_i, double _leg_length, double _conn_length, double _d, double _Rw):
    theta_i(_theta_i),
    leg_length(_leg_length),
    conn_length(_conn_length),
    d(_d),
    Rw(_Rw)
{
}

#define DBP(v) #v ": " << v << ", "
bool SRR::VehicleVelKinContainer::handle_callback(srr_msgs::CalculateVehicleVelKin::Request&  req,
                                                  srr_msgs::CalculateVehicleVelKin::Response& res)
{
    // Mark return value as undefined in case an error occurs
    res.return_code = srr_msgs::CalculateVehicleVelKin::Request::UNDEFINED;

    std::vector<geometry_msgs::Pose2D> waypoints      = req.path.waypoints;
    std::vector<geometry_msgs::Pose2D>::iterator iter = waypoints.begin();

    // The initial/current waypoint is required, check that it was provided
    if (iter == waypoints.end())
    {
        res.return_code = srr_msgs::CalculateVehicleVelKin::Request::FAILURE_NO_INIT_POSITION_GIVEN;
        return false;
    }

    // Given the inclination of the chassis, calculate the longitudinal length between the wheels
    double tfr = req.theta_front, tba = req.theta_back;
    double h = (leg_length * (std::cos(tfr) + std::cos(tba))) - (conn_length * (std::sin(tfr) + std::sin(tba)));
    double R = h / std::tan(theta_i);
    double theta_o = std::atan2(h, R + d);
    std::cout << DBP(tfr) << DBP(h) << DBP(R) << DBP(theta_o) << std::endl;

    // Declare the init and final 2D pose (x, y, and theta) and allocate space for calculated results
    geometry_msgs::Pose2D p2di = *iter, p2df;
    while (++iter != waypoints.end())
    {
        p2df = *iter;
        double xi = p2di.x, yi = p2di.y, ti = p2di.theta, xf = p2df.x, yf = p2df.y, tf = p2df.theta;
        std::cout << DBP(xi) << DBP(yi) << DBP(ti) << DBP(xf) << DBP(yf) << DBP(tf) << std::endl;

        // Solve for angle sigma, the amount of initial arc the robot should make, such that:
        //   sigma = acos(((xf-xi-l2cos(beta))/R) + cos(ti) - cos(tf)) - ti
        //    beta = gamma-alpha
        //   gamma = atan((f-b)/(e-a))
        //   alpha = atan(R/l1)
        //    l1^2 = (e-a)^2 + (f-b)^2
        //    l2^2 = l1^2 + r^2
        //       a = xi - (R * cos(ti))
        //       b = yi - (R * sin(ti))
        //       e = xf - (R * cos(tf))
        //       f = yf - (R * sin(tf))

        double e = xf - (R * cos(tf));
        double a = xi - (R * cos(ti));
        double f = yf - (R * sin(tf));
        double b = yi - (R * sin(ti));
        std::cout << DBP(e) << DBP(a) << DBP(f) << DBP(b) << std::endl;

        double e_a = e - a;
        double f_b = f - b;
        double l12 = (e_a * e_a) + (f_b * f_b);
        double l1  = std::sqrt(l12);
        double l22 = l12 + (R * R);
        double l2  = std::sqrt(l22);
        std::cout << DBP(e_a) << DBP(f_b) << DBP(l12) << DBP(l1) << DBP(l22) << DBP(l2) << std::endl;

        double gamma = std::atan(f_b / e_a);
        double alpha = std::atan(R / l1);
        double beta  = gamma - alpha;
        std::cout << DBP(gamma) << DBP(alpha) << DBP(beta) << std::endl;

        // Finally, calculate sigma
        // Then solve for angle delta, rotation to take at the end to finish with the objective orientation
        double sxpos = ((xf - xi - (l2 * std::cos(beta))) / R) + std::cos(ti) - std::cos(tf);
        double sigma = std::acos(sxpos) - ti;
        double delta = tf - ti - sigma;
        std::cout << DBP(sxpos) << DBP(sigma) << DBP(delta) << std::endl;

        // Now that sigma and delta are known, calculate the angular velocities of the wheels
        // Given the outer's wheel's desired velocity, calculate the elapsed time, use that to find the inner wheel velocity
        double f_arc_o = (R + d) * sigma;
        double f_dt = f_arc_o / (Rw * req.omega_dot_max);
        double f_arc_i = R * sigma;
        double f_omega_dot_i = f_arc_i / (Rw * f_dt);
        std::cout << DBP(f_arc_o) << DBP(f_dt) << DBP(f_arc_i) << DBP(f_omega_dot_i) << std::endl;

        // Now calculate the time elapsed for the straight portion
        // The distance to travel from (g,h) to (c,d) is the same as (a,b) to (e,f), so reuse l^2
        double m_dt = std::sqrt(l2) / (Rw * req.omega_dot_max);

        // Now repeat for the final arc
        double s_arc_o = (R + d) * delta;
        double s_dt = s_arc_o / (Rw * req.omega_dot_max);
        double s_arc_i = R * delta;
        double s_omega_dot_i = s_arc_i / (Rw * s_dt);

        // Populate message
        srr_msgs::VehicleVelKinSegment segment;
        segment.ft_lw_direction = theta_i;
        segment.ft_rw_direction = theta_o;
        segment.ft_lw_angular_velocity = f_omega_dot_i;
        segment.ft_rw_angular_velocity = req.omega_dot_max;
        segment.ft_duration = f_dt;
        segment.m_w_angular_velocities = req.omega_dot_max;
        segment.m_duration = m_dt;
        segment.st_lw_direction = theta_i;
        segment.st_rw_direction = theta_o;
        segment.st_lw_angular_velocity = s_omega_dot_i;
        segment.st_rw_angular_velocity = req.omega_dot_max;
        segment.st_duration = s_dt;

        res.segments.push_back(segment);

        p2df = p2di;
    }

    res.return_code = srr_msgs::CalculateVehicleVelKin::Request::SUCCESS;
    return true;
}
#undef DBP
